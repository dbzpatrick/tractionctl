-- PID Controller for traction control

-- Higher P: aggressive correction to error, more overshoot
-- Higher I: decrease rise time, more overshoot and oscillation
-- Higher D: reduce overshoot, potential damage for too sensitive

-- ---- TUNABLE PARAMETERS
local TARGET_SLIP        = 0.12   -- target slip ratio
local TC_GAIN_P          = 0.8    -- Proportional gain
local TC_GAIN_I          = 0.15   -- Integral gain
local TC_GAIN_D          = 0.05   -- Derivative gain
local I_bleed            = 0.9    -- Integral decay, reduce accumulated I term 
local MIN_SPEED_MS       = 0.01   -- TC inactive below 0.01m/s to avoid diving by zero
local MAX_TORQUE_NM      = 220    -- Max motor torque
local GRIP               = 1.2    -- Tire Grip Scaler
local TC_ENABLE_CHANNEL  = "TC_Enable"   -- dash switch channel name

-- ---- OTHER PARAMETERS
local WHEEL_RADIUS_M     = 0.254  -- tire radius, 10 inch
local FDR                = 3      -- Final Drive Ratio
-- ------------------------------------------------------------

-- Initialize PID state
local integral    = 0
local prev_error  = 0
local last_time   = 0

function onTimer()

    -- Read inputs
    local fl = Chan.Get("WheelSpeedFL")        -- rad/s from WSS
    local fr = Chan.Get("WheelSpeedFR")
    local rl = Chan.Get("WheelSpeedRL")
    local rr = Chan.Get("WheelSpeedRR")

    local tps        = Chan.Get("ThrottlePosition")    -- throttle percentage
    local motor_rpm  = Chan.Get("MotorRPM")            -- motor rpm
    local batt_v     = Chan.Get("BatteryVoltage")      -- V
    local batt_i     = Chan.Get("BatteryCurrent")      -- A
    local tc_on      = Chan.Get(TC_ENABLE_CHANNEL)     -- traction controller on button

    local now = Timer.GetTime()
    local dt  = now - last_time
    last_time = now
    if dt <= 0 then dt = 0.001 end

    -- Non-slip reference speed (front axle)
    local v_ref = ((fl + fr) / 2.0) * WHEEL_RADIUS_M   -- m/s

    -- Rear wheel speed
    local v_rear = ((rl + rr) / 2.0) * WHEEL_RADIUS_M

    -- Torque demanded from driver inputs
    local torque_demand = (tps / 100.0) * MAX_TORQUE_NM

    -- ---- FIRST LAYER TRACTION CONTROL by Tire Model -----------------------------------
    local Tpeak = findTpeak(v_ref, GRIP, WHEEL_RADIUS_M, FDR) -- update this function
    if torque_demand>Tpeak then torque_demand = Tpeak end


    -- ---- SECOND LAYER TRACTION CONTROL by PID Controller -----------------------------------
    local tc_reduction = 0.0

    if tc_on == 1 and v_ref > MIN_SPEED_MS then

        -- Slip ratio
        local slip = 0.0
        if v_ref > 0 then
            slip = (v_rear - v_ref) / v_ref
        end

        -- PID error
        local error = slip - TARGET_SLIP

        if error > 0 then   -- only intervene when over target slip
            integral  = integral + (error * dt)
            local derivative = (error - prev_error) / dt

            tc_reduction = (TC_GAIN_P * error)
                         + (TC_GAIN_I * integral)
                         + (TC_GAIN_D * derivative)

            -- Clamp reduction to 0–1 range
            if tc_reduction > 1.0 then tc_reduction = 1.0 end
            if tc_reduction < 0.0 then tc_reduction = 0.0 end
        else
            integral = integral * I_bleed   -- bleed integral when not active
        end

        prev_error = error

        -- log slip ratio and traction reduction data
        Chan.Set("SlipRatio", slip)
        Chan.Set("TC_Reduction", tc_reduction * 100)  -- log as %
    else
        -- TC off or below speed threshold — reset integrator
        integral   = 0
        prev_error = 0
    end

    -- ---- FINAL TORQUE OUTPUT --------------------------------
    local final_torque = torque_demand * (1.0 - tc_reduction)

    -- Hard floor — don't command negative torque unless regen intended
    if final_torque < 0 then final_torque = 0 end


    -- ---- LOGGING CHANNELS -----------------------------------
    Chan.Set("V_Reference", v_ref)
    Chan.Set("V_Rear",      v_rear)
    Chan.Set("TorqueCommand", final_torque)
    Chan.Set("PowerDemand", final_torque * (motor_rpm / 60.0) * 2.0 * math.pi) -- kinematic power output

end

function findTpeak(v_ref, GRIP, WHEEL_RADIUS_M, FDR)
    local Fpeak = 2000 * GRIP * v_ref  -- update with polyfit result with wheel speed

    local Tpeak = Fpeak * WHEEL_RADIUS_M / FDR
    return Tpeak
end