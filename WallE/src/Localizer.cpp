/*----------------------------------------------------------------------------*/
/*    Localizer.cpp - EKF-based localization service for VEX V5               */
/*    Fuses tracking wheel odometry, IMU, and dual GPS sensors                */
/*                                                                            */
/*    Coordinate conventions:                                                 */
/*    - Field: 12ft x 12ft, origin at center, ±182.88cm                       */
/*    - +X = East, +Y = North                                                 */
/*    - Heading: 0° = North (+Y), CW positive                                 */
/*    - Internal theta: radians, 0 = +Y, CW positive (same convention)        */
/*----------------------------------------------------------------------------*/

#include "Localizer.h"
#include <cmath>
#include <cstdio>

using GpsReading = DualGPS::GpsReading;

// External Brain object (defined in main.cpp)
extern vex::brain Brain;

// Constructor
Localizer::Localizer(DualGPS& gps, Drive& chassis, inertial& imu)
    : gps(gps), chassis(chassis), imu(imu),
      update_thread(nullptr),
      running(true),
      update_interval_ms(10),  // 100Hz default
    odom_initialized(false),
    gps_bootstrapped(false),
      last_debug_time(0)
{
    // Initialize tuning parameters with defaults
    Q_xy = 0.1f;              // Position process noise (cm²) - lower = trust odom more
    Q_theta = 0.001f;         // Heading process noise (rad²)
    
    // sigma_base_cm = 18.0f;    // GPS sigma at quality 100 - higher = trust GPS less
    sigma_base_cm = 10.0f;      // More GPS trust when quality is high
    // sigma_quality_90_cm = 40.0f;  // GPS sigma at quality 90 - much less trusted
    sigma_quality_90_cm = 24.0f;    // Less conservative at q=90
    // sigma_min_quality = 92;   // Ignore GPS below this quality - require high quality
    sigma_min_quality = 89;     // Accept slightly lower quality fixes
    
    wall_threshold_cm = 80.0f;    // ~32 inches from wall - detect wall proximity very early
    // wall_multiplier = 15.0f;      // Inflate sigma 15x at wall (increased from 10.0)
    wall_multiplier = 6.0f;       // Reduce over-penalizing GPS near walls
    
    gate_sigma_mult = 20.0f;      // Gate at 20 sigma (more permissive)
    gate_max_cm = 150.0f;         // Hard gate at 150cm (allow large corrections)
    
    // Variance-based trust parameters (aggressive to handle wall effects)
    // variance_low_cm = 0.5f;      // GPS is stable below this variance (lowered from 2.0)
    variance_low_cm = 1.5f;        // Less sensitive to normal jitter
    // variance_high_cm = 3.0f;     // GPS is unstable above this variance (lowered from 10.0)
    variance_high_cm = 6.0f;       // Delay heavy distrust until larger jumps
    // variance_max_mult = 10.0f;   // Max sigma multiplier from variance (increased from 5.0)
    variance_max_mult = 4.0f;      // Cap distrust inflation
    
    // Initialize variance tracking buffers
    left_history_idx = 0;
    right_history_idx = 0;
    left_history_count = 0;
    right_history_count = 0;
    left_gps_variance = 0.0f;
    right_gps_variance = 0.0f;
    for (int i = 0; i < GPS_HISTORY_SIZE; i++) {
        left_gps_history_x[i] = 0.0f;
        left_gps_history_y[i] = 0.0f;
        right_gps_history_x[i] = 0.0f;
        right_gps_history_y[i] = 0.0f;
    }
    
    // Initialize state to zero
    state[0] = 0.0f;  // x
    state[1] = 0.0f;  // y
    state[2] = 0.0f;  // theta (radians)
    
    // Initialize covariance with high uncertainty
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P[i][j] = 0.0f;
        }
    }
    P[0][0] = 10000.0f;  // 100cm sigma initially
    P[1][1] = 10000.0f;
    P[2][2] = 1.0f;      // ~57 degrees sigma initially
    
    // Initialize output
    current_pose = {0, 0, 0, 0};
    current_debug = {};
    
    // Start background update thread
    update_thread = new vex::thread(updateThread, static_cast<void*>(this));
}

Localizer::~Localizer()
{
    running = false;
    if (update_thread != nullptr) {
        update_thread->interrupt();
        vex::this_thread::sleep_for(50);
        delete update_thread;
        update_thread = nullptr;
    }
}

int Localizer::updateThread(void* arg)
{
    Localizer* instance = static_cast<Localizer*>(arg);
    while (instance->running) {
        instance->update();
        vex::this_thread::sleep_for(instance->update_interval_ms);
    }
    return 0;
}

void Localizer::setUpdateRate(int hz)
{
    if (hz > 0 && hz <= 200) {
        update_interval_ms = 1000 / hz;
    }
}

// === Angle conversion helpers ===

float Localizer::degToRad(float deg) const
{
    return deg * M_PI / 180.0f;
}

float Localizer::radToDeg(float rad) const
{
    return rad * 180.0f / M_PI;
}

float Localizer::normalizeAngle(float angle_rad) const
{
    while (angle_rad > M_PI) angle_rad -= 2.0f * M_PI;
    while (angle_rad < -M_PI) angle_rad += 2.0f * M_PI;
    return angle_rad;
}

// === Wall proximity calculation ===

float Localizer::computeWallDistance(float x_cm, float y_cm) const
{
    float dist_x = FIELD_HALF_SIZE_CM - std::fabs(x_cm);
    float dist_y = FIELD_HALF_SIZE_CM - std::fabs(y_cm);
    return (dist_x < dist_y) ? dist_x : dist_y;
}

float Localizer::computeWallFactor(float x_cm, float y_cm) const
{
    float wall_dist = computeWallDistance(x_cm, y_cm);
    if (wall_dist >= wall_threshold_cm) {
        return 0.0f;  // Far from wall
    }
    // Linear ramp from 0 (at threshold) to 1 (at wall)
    float factor = (wall_threshold_cm - wall_dist) / wall_threshold_cm;
    // Clamp factor to [0, 1]
    if (factor < 0.0f) factor = 0.0f;
    if (factor > 1.0f) factor = 1.0f;
    return factor;
}

// === GPS quality to sigma mapping ===
// Provides continuous sigma based on quality, not hard threshold

float Localizer::qualityToSigma(int quality) const
{
    if (quality >= 100) {
        return sigma_base_cm;
    }
    if (quality <= 90) {
        // Linear extrapolation below 90
        // At 90: sigma_quality_90_cm, at 80: 2x that, etc.
        float factor = (90.0f - quality) / 10.0f;
        return sigma_quality_90_cm * (1.0f + factor);
    }
    // Linear interpolation between 90 and 100
    float t = (quality - 90.0f) / 10.0f;  // 0 at q=90, 1 at q=100
    return sigma_quality_90_cm + t * (sigma_base_cm - sigma_quality_90_cm);
}

// === Variance-based GPS trust ===
// Tracks GPS reading stability over time

void Localizer::updateGpsVariance(float gps_x, float gps_y, bool is_left)
{
    // Track FRAME-TO-FRAME GPS JUMPS adjusted for odometry movement
    // This catches GPS instability regardless of EKF state corruption
    // 
    // We compute: GPS_delta - Odom_delta for each frame
    // If GPS is stable, this should be near zero even during movement
    // If GPS is jumping around, this will be large
    
    if (is_left) {
        // Get previous GPS reading (stored as raw positions now)
        float prev_gps_x = left_gps_history_x[(left_history_idx + GPS_HISTORY_SIZE - 1) % GPS_HISTORY_SIZE];
        float prev_gps_y = left_gps_history_y[(left_history_idx + GPS_HISTORY_SIZE - 1) % GPS_HISTORY_SIZE];
        
        // Store current raw GPS position
        left_gps_history_x[left_history_idx] = gps_x;
        left_gps_history_y[left_history_idx] = gps_y;
        left_history_idx = (left_history_idx + 1) % GPS_HISTORY_SIZE;
        if (left_history_count < GPS_HISTORY_SIZE) {
            left_history_count++;
        }
        
        // Calculate frame-to-frame GPS jump (if we have previous reading)
        if (left_history_count >= 2) {
            float gps_delta_x = gps_x - prev_gps_x;
            float gps_delta_y = gps_y - prev_gps_y;
            
            // GPS jump magnitude (ideally matches odometry delta, but we use raw jump)
            // Large jumps = unstable GPS
            float jump = std::sqrt(gps_delta_x * gps_delta_x + gps_delta_y * gps_delta_y);
            
            // Use exponential moving average with HIGH alpha for fast reaction
            // Alpha = 0.8 means 80% weight on NEW reading - reacts very quickly
            float alpha = 0.8f;
            if (left_history_count == 2) {
                left_gps_variance = jump;  // Initialize
            } else {
                // Take max of current jump vs decayed variance - big jumps trigger immediately
                float decayed = alpha * jump + (1.0f - alpha) * left_gps_variance;
                left_gps_variance = (jump > decayed) ? jump : decayed;
            }
        }
    } else {
        // Get previous GPS reading
        float prev_gps_x = right_gps_history_x[(right_history_idx + GPS_HISTORY_SIZE - 1) % GPS_HISTORY_SIZE];
        float prev_gps_y = right_gps_history_y[(right_history_idx + GPS_HISTORY_SIZE - 1) % GPS_HISTORY_SIZE];
        
        // Store current raw GPS position
        right_gps_history_x[right_history_idx] = gps_x;
        right_gps_history_y[right_history_idx] = gps_y;
        right_history_idx = (right_history_idx + 1) % GPS_HISTORY_SIZE;
        if (right_history_count < GPS_HISTORY_SIZE) {
            right_history_count++;
        }
        
        // Calculate frame-to-frame GPS jump
        if (right_history_count >= 2) {
            float gps_delta_x = gps_x - prev_gps_x;
            float gps_delta_y = gps_y - prev_gps_y;
            
            float jump = std::sqrt(gps_delta_x * gps_delta_x + gps_delta_y * gps_delta_y);
            
            // Use exponential moving average with HIGH alpha for fast reaction
            float alpha = 0.8f;
            if (right_history_count == 2) {
                right_gps_variance = jump;
            } else {
                // Take max of current jump vs decayed variance - big jumps trigger immediately
                float decayed = alpha * jump + (1.0f - alpha) * right_gps_variance;
                right_gps_variance = (jump > decayed) ? jump : decayed;
            }
        }
    }
}

float Localizer::getVarianceSigmaMult(bool is_left) const
{
    float variance = is_left ? left_gps_variance : right_gps_variance;
    
    if (variance <= variance_low_cm) {
        return 1.0f;  // GPS is stable, trust it normally
    }
    if (variance >= variance_high_cm) {
        return variance_max_mult;  // GPS is very unstable, trust it much less
    }
    
    // Linear interpolation between low and high thresholds
    float t = (variance - variance_low_cm) / (variance_high_cm - variance_low_cm);
    return 1.0f + t * (variance_max_mult - 1.0f);
}

// === EKF Prediction step ===
// Propagates state based on odometry motion

void Localizer::predict(float dx_cm, float dy_cm, float dtheta_rad)
{
    // State prediction (simple dead reckoning)
    // VEX odom already gives world-frame deltas, so just add directly
    state[0] += dx_cm;
    state[1] += dy_cm;
    state[2] += dtheta_rad;
    state[2] = normalizeAngle(state[2]);
    
    // Covariance prediction: P = P + Q
    // Simplified: just add process noise to diagonal
    P[0][0] += Q_xy;
    P[1][1] += Q_xy;
    P[2][2] += Q_theta;
}

// === EKF Update step for GPS measurement ===
// Updates state based on GPS position measurement (x, y only)

void Localizer::updateGPS(float z_x, float z_y, float sigma)
{
    // Measurement model: H = [[1,0,0], [0,1,0]]
    // We observe x and y directly
    
    float R = sigma * sigma;  // Measurement variance
    
    // Innovation (residual)
    float y0 = z_x - state[0];
    float y1 = z_y - state[1];
    
    // Innovation covariance: S = H*P*H' + R
    // S is 2x2: [[P00+R, P01], [P10, P11+R]]
    float S00 = P[0][0] + R;
    float S01 = P[0][1];
    float S10 = P[1][0];
    float S11 = P[1][1] + R;
    
    // Kalman gain: K = P*H' * inv(S)
    // K is 3x2
    // inv(S) for 2x2: 1/det * [[S11, -S01], [-S10, S00]]
    float det = S00 * S11 - S01 * S10;
    if (std::fabs(det) < 1e-6f) return;  // Singular, skip update
    
    float invDet = 1.0f / det;
    float Si00 = S11 * invDet;
    float Si01 = -S01 * invDet;
    float Si10 = -S10 * invDet;
    float Si11 = S00 * invDet;
    
    // K = P * H' * Si
    // P*H' = [[P00, P01], [P10, P11], [P20, P21]]
    float K00 = P[0][0] * Si00 + P[0][1] * Si10;
    float K01 = P[0][0] * Si01 + P[0][1] * Si11;
    float K10 = P[1][0] * Si00 + P[1][1] * Si10;
    float K11 = P[1][0] * Si01 + P[1][1] * Si11;
    float K20 = P[2][0] * Si00 + P[2][1] * Si10;
    float K21 = P[2][0] * Si01 + P[2][1] * Si11;
    
    // State update: x = x + K*y
    state[0] += K00 * y0 + K01 * y1;
    state[1] += K10 * y0 + K11 * y1;
    state[2] += K20 * y0 + K21 * y1;
    state[2] = normalizeAngle(state[2]);
    
    // Covariance update: P = (I - K*H) * P
    // Using Joseph form for numerical stability would be better, but simplified here
    float KH[3][3] = {
        {K00, K01, 0},
        {K10, K11, 0},
        {K20, K21, 0}
    };
    
    float P_new[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P_new[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                // (I - K*H)_ik where H = [[1,0,0],[0,1,0]]
                float IKH_ik = (i == k ? 1.0f : 0.0f);
                if (k < 2) IKH_ik -= KH[i][k];
                P_new[i][j] += IKH_ik * P[k][j];
            }
        }
    }
    
    // Copy back
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P[i][j] = P_new[i][j];
        }
    }
}

// === Main update loop ===

void Localizer::update()
{
    // Get current odometry from chassis (JAR-Template, in inches)
    float odom_x_in = chassis.get_X_position();
    float odom_y_in = chassis.get_Y_position();
    float heading_deg = imu.heading(degrees);  // Use IMU for heading
    
    // Convert heading to radians (0=North, CW positive)
    float heading_rad = degToRad(heading_deg);
    
    // Initialize odometry tracking on first run
    if (!odom_initialized) {
        prev_odom_x_in = odom_x_in;
        prev_odom_y_in = odom_y_in;
        prev_heading_deg = heading_deg;
        odom_initialized = true;
        return;
    }
    
    // === PREDICTION STEP ===
    // Compute odometry deltas (in inches, convert to cm)
    float dx_in = odom_x_in - prev_odom_x_in;
    float dy_in = odom_y_in - prev_odom_y_in;
    float dx_cm = dx_in * 2.54f;
    float dy_cm = dy_in * 2.54f;
    
    // Heading delta
    float dtheta_deg = heading_deg - prev_heading_deg;
    // Handle wrap-around
    while (dtheta_deg > 180.0f) dtheta_deg -= 360.0f;
    while (dtheta_deg < -180.0f) dtheta_deg += 360.0f;
    float dtheta_rad = degToRad(dtheta_deg);
    
    // Update previous values
    prev_odom_x_in = odom_x_in;
    prev_odom_y_in = odom_y_in;
    prev_heading_deg = heading_deg;
    
    // Predict state
    predict(dx_cm, dy_cm, dtheta_rad);
    
    // Sync theta state with IMU (IMU is authoritative for heading)
    // This prevents heading drift while still allowing EKF to track x,y
    state[2] = heading_rad;
    
    // === GPS UPDATE STEP ===
    
    // Prepare debug info
    LocalizerDebug debug = {};
    debug.left_gps_used = false;
    debug.right_gps_used = false;
    debug.left_gps_gated = false;
    debug.right_gps_gated = false;
    
    // Compute wall factor based on current estimated position
    float wall_dist = computeWallDistance(state[0], state[1]);
    float wall_factor = computeWallFactor(state[0], state[1]);
    debug.wall_distance_cm = wall_dist;
    debug.wall_factor = wall_factor;
    
    // Get GPS readings
    GpsReading left_gps, right_gps;
    bool have_left = gps.getLeft(left_gps);
    bool have_right = gps.getRight(right_gps);
    
    debug.left_quality = left_gps.quality;
    debug.right_quality = right_gps.quality;
    
    // Determine if only one GPS is usable (single-GPS mode)
    // In this mode, we trust the remaining GPS less since there's no cross-validation
    bool left_usable = have_left && left_gps.valid && left_gps.quality >= sigma_min_quality;
    bool right_usable = have_right && right_gps.valid && right_gps.quality >= sigma_min_quality;
    bool single_gps_mode = (left_usable && !right_usable) || (!left_usable && right_usable);
    float single_gps_sigma_mult = 10.0f;  // Inflate sigma heavily when only one GPS (single GPS often has bias)

    // Bootstrap EKF state from GPS once it becomes usable.
    // Prevents the filter from staying stuck near (0,0) due to strict residual gating.
    if (!gps_bootstrapped && (left_usable || right_usable)) {
        float bootstrap_x = state[0];
        float bootstrap_y = state[1];

        if (left_usable && right_usable) {
            bootstrap_x = (left_gps.x_cm + right_gps.x_cm) * 0.5f;
            bootstrap_y = (left_gps.y_cm + right_gps.y_cm) * 0.5f;
        } else if (left_usable) {
            bootstrap_x = left_gps.x_cm;
            bootstrap_y = left_gps.y_cm;
        } else {
            bootstrap_x = right_gps.x_cm;
            bootstrap_y = right_gps.y_cm;
        }

        state[0] = bootstrap_x;
        state[1] = bootstrap_y;
        state[2] = degToRad(gps.heading());

        P[0][0] = 100.0f;  // 10cm sigma after bootstrap
        P[1][1] = 100.0f;
        P[2][2] = 0.05f;

        prev_odom_x_in = odom_x_in;
        prev_odom_y_in = odom_y_in;
        prev_heading_deg = heading_deg;
        odom_initialized = true;
        gps_bootstrapped = true;
    }
    
    // === TRANSITION PROTECTION ===
    // Calculate dynamic residual cap based on conditions
    // Base cap is 4cm (tight), gets even tighter when conditions are poor
    // float base_residual_cap = 4.0f;
    float base_residual_cap = 12.0f;
    
    // Tighten cap near walls (wall_factor 0→1 maps to cap 4→2cm)
    // float wall_adjusted_cap = base_residual_cap - (wall_factor * 2.0f);
    float wall_adjusted_cap = base_residual_cap - (wall_factor * 4.0f);
    if (wall_adjusted_cap < 8.0f) wall_adjusted_cap = 8.0f;
    
    // Get the quality of the working GPS in single-GPS mode
    int working_gps_quality = 100;
    if (single_gps_mode) {
        working_gps_quality = left_usable ? left_gps.quality : right_gps.quality;
    }
    
    // Tighten cap if quality is degrading (q100→5cm, q90→4cm, q80→3cm)
    // float quality_factor = (100 - working_gps_quality) / 10.0f;  // 0 at q100, 1 at q90, 2 at q80
    float quality_factor = (100 - working_gps_quality) / 20.0f;    // Less aggressive quality penalty
    float quality_adjusted_cap = wall_adjusted_cap - quality_factor;
    if (quality_adjusted_cap < 8.0f) quality_adjusted_cap = 8.0f;
    
    // Final dynamic cap for single-GPS mode
    float dynamic_single_gps_max_residual = quality_adjusted_cap;
    
    // ABSOLUTE maximum residual for ANY GPS update (even when both GPS work)
    // If a GPS reading is more than 10cm from state, it's likely wrong regardless of mode
    float absolute_max_residual = gps_bootstrapped ? 80.0f : 200.0f;
    
    // Process LEFT GPS
    if (left_usable) {
        // Update variance tracking with new reading
        updateGpsVariance(left_gps.x_cm, left_gps.y_cm, true);
        
        float variance_mult = getVarianceSigmaMult(true);
        debug.variance_left_cm = left_gps_variance;
        debug.variance_mult_left = variance_mult;
        
        // HARD GATE: If variance (frame-to-frame jump) is too high, skip GPS entirely
        // This prevents unstable GPS from corrupting the state
        // At 100Hz, stable GPS should show <1cm jumps; 3cm+ means instability
        // float variance_gate_threshold = 3.0f;  // Skip GPS if jump > 3cm
        float variance_gate_threshold = 5.0f;    // Skip GPS only on larger jumps
        if (left_gps_variance > variance_gate_threshold && left_history_count >= 2) {
            debug.sigma_left_cm = 999.0f;
            debug.residual_left_cm = 999.0f;
            debug.left_gps_gated = true;
        } else {
            float sigma = qualityToSigma(left_gps.quality);
            sigma *= (1.0f + wall_factor * wall_multiplier);
            
            // In single-GPS mode, trust this reading less
            if (single_gps_mode) {
                sigma *= single_gps_sigma_mult;
            }
            
            // Apply variance-based trust: if readings are fluctuating, trust less
            sigma *= variance_mult;
            
            debug.sigma_left_cm = sigma;
            
            // Compute residual for gating
            float residual = std::sqrt(
                (left_gps.x_cm - state[0]) * (left_gps.x_cm - state[0]) +
                (left_gps.y_cm - state[1]) * (left_gps.y_cm - state[1])
            );
            debug.residual_left_cm = residual;
            
            // Gate check
            float gate_threshold = (sigma * gate_sigma_mult < gate_max_cm) ? sigma * gate_sigma_mult : gate_max_cm;
            
            // ABSOLUTE CAP: Gate any GPS reading that's too far from state
            // This catches outliers even when both GPS are "working"
            if (residual > absolute_max_residual) {
                debug.left_gps_gated = true;  // Definitely wrong - gate it
            }
            // SINGLE-GPS CAP: Use tighter dynamic cap when only one GPS works
            else if (single_gps_mode && residual > dynamic_single_gps_max_residual) {
                debug.left_gps_gated = true;  // Gate - likely biased
            } 
            // Normal gating
            else if (residual < gate_threshold) {
                updateGPS(left_gps.x_cm, left_gps.y_cm, sigma);
                debug.left_gps_used = true;
            } else {
                debug.left_gps_gated = true;
            }
        }
    } else {
        debug.sigma_left_cm = 999.0f;
        debug.residual_left_cm = 999.0f;
        debug.variance_left_cm = left_gps_variance;
        debug.variance_mult_left = 1.0f;
    }
    
    // Process RIGHT GPS
    if (right_usable) {
        // Update variance tracking with new reading
        updateGpsVariance(right_gps.x_cm, right_gps.y_cm, false);
        
        float variance_mult = getVarianceSigmaMult(false);
        debug.variance_right_cm = right_gps_variance;
        debug.variance_mult_right = variance_mult;
        
        // HARD GATE: If variance (frame-to-frame jump) is too high, skip GPS entirely
        // This prevents unstable GPS from corrupting the state
        // At 100Hz, stable GPS should show <1cm jumps; 3cm+ means instability
        // float variance_gate_threshold = 3.0f;  // Skip GPS if jump > 3cm
        float variance_gate_threshold = 5.0f;    // Skip GPS only on larger jumps
        if (right_gps_variance > variance_gate_threshold && right_history_count >= 2) {
            debug.sigma_right_cm = 999.0f;
            debug.residual_right_cm = 999.0f;
            debug.right_gps_gated = true;
        } else {
            float sigma = qualityToSigma(right_gps.quality);
            sigma *= (1.0f + wall_factor * wall_multiplier);
            
            // In single-GPS mode, trust this reading less
            if (single_gps_mode) {
                sigma *= single_gps_sigma_mult;
            }
            
            // Apply variance-based trust: if readings are fluctuating, trust less
            sigma *= variance_mult;
            
            debug.sigma_right_cm = sigma;
            
            // Compute residual for gating
            float residual = std::sqrt(
                (right_gps.x_cm - state[0]) * (right_gps.x_cm - state[0]) +
                (right_gps.y_cm - state[1]) * (right_gps.y_cm - state[1])
            );
            debug.residual_right_cm = residual;
            
            // Gate check
            float gate_threshold = (sigma * gate_sigma_mult < gate_max_cm) ? sigma * gate_sigma_mult : gate_max_cm;
            
            // ABSOLUTE CAP: Gate any GPS reading that's too far from state
            // This catches outliers even when both GPS are "working"
            if (residual > absolute_max_residual) {
                debug.right_gps_gated = true;  // Definitely wrong - gate it
            }
            // SINGLE-GPS CAP: Use tighter dynamic cap when only one GPS works
            else if (single_gps_mode && residual > dynamic_single_gps_max_residual) {
                debug.right_gps_gated = true;  // Gate - likely biased
            }
            // Normal gating
            else if (residual < gate_threshold) {
                updateGPS(right_gps.x_cm, right_gps.y_cm, sigma);
                debug.right_gps_used = true;
            } else {
                debug.right_gps_gated = true;
            }
        }
    } else {
        debug.sigma_right_cm = 999.0f;
        debug.residual_right_cm = 999.0f;
        debug.variance_right_cm = right_gps_variance;
        debug.variance_mult_right = 1.0f;
    }
    
    // Update covariance debug
    debug.sigma_x_cm = std::sqrt(P[0][0]);
    debug.sigma_y_cm = std::sqrt(P[1][1]);
    debug.sigma_theta_deg = radToDeg(std::sqrt(P[2][2]));
    
    // === OUTPUT ===
    // Convert state to output pose (mutex protected)
    float out_heading_deg = radToDeg(state[2]);
    while (out_heading_deg < 0) out_heading_deg += 360.0f;
    while (out_heading_deg >= 360.0f) out_heading_deg -= 360.0f;
    
    pose_mutex.lock();
    current_pose.x_cm = state[0];
    current_pose.y_cm = state[1];
    current_pose.heading_deg = out_heading_deg;
    current_pose.timestamp_ms = Brain.Timer.system();
    current_debug = debug;
    pose_mutex.unlock();
    
    // Debug logging (throttled to 1Hz)
    #if LOCALIZER_DEBUG
    uint32_t now = Brain.Timer.system();
    if (now - last_debug_time >= 1000) {
        last_debug_time = now;
        printf("LOC: (%.1f,%.1f)@%.1f sig=(%.1f,%.1f) wall=%.1f L:%d R:%d\n",
               state[0], state[1], out_heading_deg,
               debug.sigma_x_cm, debug.sigma_y_cm,
               wall_dist,
               debug.left_gps_used ? 1 : 0,
               debug.right_gps_used ? 1 : 0);
    }
    #endif
}

// === Public interface ===

Pose Localizer::getPose() const
{
    pose_mutex.lock();
    Pose result = current_pose;
    pose_mutex.unlock();
    return result;
}

LocalizerDebug Localizer::getDebug() const
{
    pose_mutex.lock();
    LocalizerDebug result = current_debug;
    pose_mutex.unlock();
    return result;
}

void Localizer::resetPose(float x_cm, float y_cm, float heading_deg)
{
    pose_mutex.lock();
    
    state[0] = x_cm;
    state[1] = y_cm;
    state[2] = degToRad(heading_deg);
    
    // Reset covariance to small values (high confidence in reset pose)
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P[i][j] = 0.0f;
        }
    }
    P[0][0] = 25.0f;   // 5cm sigma
    P[1][1] = 25.0f;   // 5cm sigma
    P[2][2] = 0.01f;   // ~5 degrees sigma
    
    // Also sync odometry and IMU
    chassis.set_coordinates(x_cm / 2.54f, y_cm / 2.54f, heading_deg);
    imu.setHeading(heading_deg, degrees);
    
    // Reset odom tracking
    prev_odom_x_in = chassis.get_X_position();
    prev_odom_y_in = chassis.get_Y_position();
    prev_heading_deg = heading_deg;
    odom_initialized = true;
    gps_bootstrapped = true;
    
    // Update output
    current_pose.x_cm = x_cm;
    current_pose.y_cm = y_cm;
    current_pose.heading_deg = heading_deg;
    current_pose.timestamp_ms = Brain.Timer.system();
    
    pose_mutex.unlock();
}

void Localizer::initWithStartPose(float x_cm, float y_cm, float heading_deg)
{
    resetPose(x_cm, y_cm, heading_deg);
}

bool Localizer::initFromGPS(int timeout_ms)
{
    uint32_t start_time = Brain.Timer.system();
    
    while (Brain.Timer.system() - start_time < (uint32_t)timeout_ms) {
        GpsReading left, right;
        gps.getLeft(left);
        gps.getRight(right);
        
        // Wait for both GPS to have good quality
        if (left.valid && right.valid && 
            left.quality >= 90 && right.quality >= 90) {
            
            // Use center position
            float x = (left.x_cm + right.x_cm) / 2.0f;
            float y = (left.y_cm + right.y_cm) / 2.0f;
            // Use GPS heading average (IMU should already be synced, but use GPS to be safe)
            float lH = left.heading_deg;
            float rH = right.heading_deg;
            float diff = rH - lH;
            if (diff > 180) diff -= 360;
            if (diff < -180) diff += 360;
            float heading = lH + diff / 2.0f;
            if (heading < 0) heading += 360;
            if (heading >= 360) heading -= 360;
            
            resetPose(x, y, heading);
            return true;
        }
        
        vex::this_thread::sleep_for(50);
    }
    
    // Timeout - use whatever GPS we have
    GpsReading left, right;
    gps.getLeft(left);
    gps.getRight(right);
    
    if (left.valid && left.quality >= 90) {
        resetPose(left.x_cm, left.y_cm, left.heading_deg);
        return true;
    }
    if (right.valid && right.quality >= 90) {
        resetPose(right.x_cm, right.y_cm, right.heading_deg);
        return true;
    }
    
    return false;  // Failed to initialize from GPS
}
