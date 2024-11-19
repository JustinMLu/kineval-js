function update_pendulum_state(numerical_integrator, pendulum, dt, gravity) {
    // integrate pendulum state forward in time by dt
    // please use names 'pendulum.angle', 'pendulum.angle_previous', etc. in else codeblock between line 28-30

    if (typeof numerical_integrator === "undefined")
        numerical_integrator = "none";

    if (numerical_integrator === "euler") {
    // STENCIL: a correct Euler integrator is REQUIRED for assignment
        pendulum.angle_previous = pendulum.angle;
        pendulum.angle = pendulum.angle + pendulum.angle_dot * dt;
        pendulum.angle_dot = pendulum.angle_dot + pendulum.angle_dot_dot * dt;
    }

    else if (numerical_integrator === "verlet") {
    // STENCIL: basic Verlet integration
    
    }

    else if (numerical_integrator === "velocity verlet") {
    // STENCIL: a correct velocity Verlet integrator is REQUIRED for assignment
        pendulum.angle_previous = pendulum.angle;
        
        pendulum.angle = pendulum.angle + pendulum.angle_dot * dt + 0.5 * pendulum.angle_dot_dot * Math.pow(dt, 2);
        pendulum.angle_dot = pendulum.angle_dot + 0.5 * (pendulum.angle_dot_dot + pendulum_acceleration(pendulum, gravity)) * dt;
    }

    else if (numerical_integrator === "runge-kutta") {
    // STENCIL: Runge-Kutta 4 integrator
    } 

    else {
        pendulum.angle_previous = pendulum.angle;
        pendulum.angle = (pendulum.angle+Math.PI/180)%(2*Math.PI);
        pendulum.angle_dot = (pendulum.angle-pendulum.angle_previous)/dt;
        numerical_integrator = "none";
    }

    return pendulum;
}

function pendulum_acceleration(pendulum, gravity) {
    // STENCIL: return acceleration(s) system equation(s) of motion 
    return -1 * (gravity / pendulum.length) * Math.sin(pendulum.angle) + 
            (pendulum.control / (pendulum.mass * Math.pow(pendulum.length, 2)));
    
}

function init_verlet_integrator(pendulum, t, gravity) {
    // STENCIL: for verlet integration, a first step in time is needed
    // return: updated pendulum state and time
    return [pendulum, t];
}

function set_PID_parameters(pendulum) {
    // STENCIL: change pid parameters
    pendulum.servo = {kp:140, kd:50, ki:10};  // no control
    return pendulum;
}

function PID(pendulum, accumulated_error, dt) {
    // STENCIL: implement PID controller
    // return: updated output in pendulum.control and accumulated_error
    var error = pendulum.desired - pendulum.angle;
    var prev_error = pendulum.desired - pendulum.angle_previous;

    accumulated_error = accumulated_error + error;

    pendulum.control = (pendulum.servo.kp * error) + (pendulum.servo.ki * (accumulated_error)) + (pendulum.servo.kd * ((error - prev_error) / dt));

    return [pendulum, accumulated_error];
}