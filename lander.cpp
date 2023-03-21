// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"

void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
    //VERTICAL DESCENT LANDING
    if (!infiniteFuelOn) {
        FUEL_RATE_AT_MAX_THRUST = 0.5;
    }
    //K_h = 0.008; // A positive constant - seems to affect how early or late the throttle is initally used. A higher value corresponds to less fuel used, a value too high means it starts slowing down too late, so it crashes.
    double K_p = 1; // A positive constant
    double delta = lander_mass() * (GRAVITY * MARS_MASS) / (position.abs2() * MAX_THRUST); // A positive constant between 0 and 1
    double height = position.abs() - MARS_RADIUS; // Lander altitude
    double error = -(0.5 + (K_h * height) + (velocity * position.norm())); // Error term
    double P_out = K_p * error;

    if (scenario != 8) {
        if (P_out <= -delta) {
            throttle = 0;
        }
        else if (P_out > -delta && P_out < (1 - delta)) {
            throttle = delta + P_out;
        }
        else {
            throttle = 1;
        }
        // modifications to counteract constant acceleration due to wind
        vector3d velocity_from_positions;
        velocity_from_positions = velocity;
        vector3d last_position = position - delta_t * velocity_from_positions;
        vector3d av_p = (position + last_position).norm();
        if (delta_t != 0.0) velocity_from_positions = (position - last_position) / delta_t;
        else velocity_from_positions = vector3d(0.0, 0.0, 0.0);
        double climb_speed = velocity_from_positions * av_p;
        double ground_speed = (velocity_from_positions - climb_speed * av_p).abs();

        if (!wind) {
            stabilized_attitude_angle = 0.0;
            attitude_stabilization();
        }
        else {
            stabilized_attitude = false;
            if (ground_speed >= 0.8) {
                stabilized_attitude_angle = 10 * M_PI / 180;
                attitude_stabilization();
            }
            else if (ground_speed > -0.8 && ground_speed < 0.8) {
                stabilized_attitude_angle = 0 * M_PI / 180;
                attitude_stabilization();
            }
            else {
                stabilized_attitude_angle = -10 * M_PI / 180;
                attitude_stabilization();
            }
        }

        if (safe_to_deploy_parachute() && height < 150000) {
            parachute_status = DEPLOYED;
        }

        /*
        //output height and velocity * position.norm() at each time step into python.
        ofstream file_out;
        file_out.open("C:/Users/soham/Documents/lander/results.txt", std::ios::app);
        if (file_out) {
            file_out << simulation_time << " " << height << " " << velocity * position.norm() << endl;
        }
        else {
            cout << "Could not open trajectory file for writing" << endl;
        }
        */
    }
    else {
        if (height < tuningHeight) {
            //K_h = 0.008; // A positive constant - seems to affect how early or late the throttle is initally used. A higher value corresponds to less fuel used, a value too high means it starts slowing down too late, so it crashes.
            double K_p = 1; // A positive constant
            double delta = lander_mass() * (GRAVITY * MARS_MASS) / (position.abs2() * MAX_THRUST); // A positive constant between 0 and 1
            double height = position.abs() - MARS_RADIUS; // Lander altitude
            double error = -(0.5 + (K_h * height) + (velocity * position.norm())); // Error term
            double P_out = K_p * error;

            if (P_out <= -delta) {
                throttle = 0;
            }
            else if (P_out > -delta && P_out < (1 - delta)) {
                throttle = delta + P_out;
            }
            else {
                throttle = 1;
            }
            // modifications to counteract constant acceleration due to wind
            vector3d velocity_from_positions;
            velocity_from_positions = velocity;
            vector3d last_position = position - delta_t * velocity_from_positions;
            vector3d av_p = (position + last_position).norm();
            if (delta_t != 0.0) velocity_from_positions = (position - last_position) / delta_t;
            else velocity_from_positions = vector3d(0.0, 0.0, 0.0);
            double climb_speed = velocity_from_positions * av_p;
            double ground_speed = (velocity_from_positions - climb_speed * av_p).abs();

            if (!wind) {
                stabilized_attitude_angle = 0.0;
                attitude_stabilization();
            }
            else {
                stabilized_attitude = false;
                if (ground_speed >= 0.8) {
                    stabilized_attitude_angle = 10 * M_PI / 180;
                    attitude_stabilization();
                }
                else if (ground_speed > -0.8 && ground_speed < 0.8) {
                    stabilized_attitude_angle = 0 * M_PI / 180;
                    attitude_stabilization();
                }
                else {
                    stabilized_attitude_angle = -10 * M_PI / 180;
                    attitude_stabilization();
                }
            }

            if (safe_to_deploy_parachute() && height < 150000) {
                parachute_status = DEPLOYED;
            }
        }
    }
}

void autopilot_landing_sequence(double kh)
// Autopilot to adjust the engine throttle, parachute and attitude control, when re entering orbit.
{
    double K_h = 0.0002; // A positive constant - seems to affect how early or late the throttle is initally used. A higher value corresponds to less fuel used, a value too high means it starts slowing down too late, so it crashes.
    double K_p = 1; // A positive constant
    double delta = lander_mass() * (GRAVITY * MARS_MASS) / (position.abs2() * MAX_THRUST); // A positive constant between 0 and 1
    double height = position.abs() - MARS_RADIUS; // Lander altitude
    double error = -(1000 + (kh * height) - (kh * 200000) + (velocity * position.norm())); // Error term
    double P_out = K_p * error;

    if (P_out <= -delta) {
        throttle = 0;
    }
    else if (P_out > -delta && P_out < (1 - delta)) {
        throttle = delta + P_out;
    }
    else {
        throttle = 1;
    }
}

void hohmann_transfer(void) {
    double height = position.abs() - MARS_RADIUS;
    /*
    //recreate the climb speed variable so that the autopilot knows when the lander is at perigee or apogee. // NOTE: Can also use a static variable and keep updating if new position is higher than previous one, which is shorter code but oh well, Ive already done this version and didnt end up needing it.
    static double apogee = 0.0;
    static double perigee = 0.0;
    vector3d velocity_from_positions;
    velocity_from_positions = velocity;
    vector3d last_position = position - delta_t * velocity_from_positions;
    vector3d av_p = (position + last_position).norm();
    if (delta_t != 0.0) velocity_from_positions = (position - last_position) / delta_t;
    else velocity_from_positions = vector3d(0.0, 0.0, 0.0);
    double climb_speed = velocity_from_positions * av_p;

    //need values of apogee and perigee to calculate change in velocity from elliptical to circular orbit

    if ((climb_speed < 0.05 && climb_speed > -0.05) && position.abs() < 4e6) {
        if (perigee == 0.0) {
            perigee = position.abs();
        }
        cout << perigee << " Perigee" << endl;
    }
    else if ((climb_speed < 0.02 && climb_speed > -0.02) && position.abs() > 4e6) {
        if (apogee == 0.0) {
            apogee = position.abs();
        }        
        cout << apogee << " Apogee" << endl;
    }
    if (position.abs() == apogee) cout << velocity << endl;
    //calculate impulse required to switch orbits
    */

    //when velocity is perpendicular to position, ie velocity y / x = 0.0, then add an impulse and orbital shape will change.
     // changing this can change the eccentricity of the final orbit
    if (velocity.y < boundary && velocity.y > -boundary) {
        stabilized_attitude = false;
        stabilized_attitude_angle = -90 * M_PI / 180 ;
        attitude_stabilization();
        throttle = 1;
        if (velocity.y < (- boundary + 0.5) && velocity.y > -boundary) {
            hohmann_enabled = false;
            throttle = 0;
        }
    }
    else {
        throttle = 0;
        stabilized_attitude = false;
    }

}

void orbital_injection(void) {
    FUEL_RATE_AT_MAX_THRUST = 0.0;
    double height = position.abs() - MARS_RADIUS;

    if (height <= 90000) {
        throttle = 1;
        attitude_stabilization();
    }

    else if (height > 90000 && height < 450000) {
        if (velocity.x < 2300) {
            throttle = 1;
            stabilized_attitude = false;
            stabilized_attitude_angle = -60 * M_PI / 180;
            attitude_stabilization();
        }
        else if (velocity.x >= 2300 && velocity.x < 3300) {
            // to put into orbit now you need to slowly change attitude angle to 90.
            stabilized_attitude = false;
            stabilized_attitude_angle = (- 0.03 * velocity.x + 9) * M_PI / 180;
            attitude_stabilization();
        }
    } 
    else if (height >= 450000) {
        stabilized_attitude_angle = -90 * M_PI / 180;
        attitude_stabilization();
        throttle = 0;
        hohmann_enabled = true;
        orbital_injection_enabled = false;
    }
}

void descending_sequence(void) {
    double height = position.abs() - MARS_RADIUS;
    stabilized_attitude = false;
    //whatever height its at, it needs to get to 200km and arrive at a decent speed, so then autopilot can land it.    
   
    if (height < 200000) { //|| velocity.abs() < 50) {
        stabilized_attitude = false;
        descending_sequence_enabled = false;
        autopilot_enabled = true;
    }
    else {
        autopilot_landing_sequence(0.000005);
    }
}

void orbital_reentry(void) {
    //face the lander opposite to the direction of velocity
    //thrust to 1 till velocity is sufficiently low
    FUEL_RATE_AT_MAX_THRUST = 0.0;
    double height = position.abs() - MARS_RADIUS;
    stabilized_attitude = false;
    stabilized_attitude_angle = 90 * M_PI / 180;
    attitude_stabilization();
    
    if (scenario == 6 || scenario == 0 || scenario == 2) {
        throttle = 0.7;
    }
    else {
        throttle = 0.5;
    }
        
    if ((velocity.norm() * position.norm()) < -0.95 && (velocity.norm() * position.norm()) > -1.05) {
        stabilized_attitude = false;
        stabilized_attitude_angle = 0;
        attitude_stabilization();
        double heightSituation;
        if (scenario == 6) {
            heightSituation = 2000000;
        }
        else {
            heightSituation = 1100000;
        }

        if (height < heightSituation) {
            descending_sequence_enabled = true;
            orbital_reentry_enabled = false;

            /*
            stabilized_attitude = false;
            stabilized_attitude_angle = 0;
            attitude_stabilization();
            throttle = 1;
            if (height < 200000) {
                cout << velocity << endl;
                stabilized_attitude = false;
                orbital_reentry_enabled = false;
                descending_sequence_enabled = true;
                autopilot();

            }
            */
        }
        else {
            throttle = 0;
        }
    }
    
    

    //at a certain height enable autopilot.
}

double drag_Coef(void) {
    double dragCoefficient = DRAG_COEF_LANDER;
    if (parachute_status == DEPLOYED) {
        dragCoefficient += DRAG_COEF_CHUTE;
    }
    return dragCoefficient;
}

double lander_mass(void) {
    return (UNLOADED_LANDER_MASS + fuel * FUEL_CAPACITY * FUEL_DENSITY);
}

double lander_area(void) {
    double area = 3.1415 * (LANDER_SIZE*LANDER_SIZE) ;
    if (parachute_status == DEPLOYED) {
        area += 4*LANDER_SIZE*LANDER_SIZE*4;
    }
    return area;
}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{  
    static vector3d previous_position;
    vector3d new_position;
    
    vector3d F_drag = -0.5 * atmospheric_density(position) * drag_Coef() * velocity.abs2() * velocity.norm() * lander_area();
    vector3d F_thrust = thrust_wrt_world();
    vector3d F_weight = -position.norm() * (GRAVITY * MARS_MASS * lander_mass())/ position.abs2();
    
    // Wind!
    vector3d F_wind = vector3d(0, 0, 0);
    if (wind_enabled) {
        F_wind = -wind();
    }

    vector3d acceleration = (F_drag + F_thrust + F_weight + F_wind)/lander_mass(); // work out resultant force on lander
    
    if (simulation_time == 0.0) {
        //do an Euler update for the first iteration
        new_position = position + (delta_t * velocity);
        velocity = velocity + (delta_t * acceleration); 
    }
    
    else {
        //do a Verlet update on all subsequent iterations
        
        if (position.abs() >= MARS_RADIUS) {
            new_position = (2 * position) - previous_position + (delta_t * delta_t * acceleration);
            velocity = (1 / delta_t) * (new_position - position);
        }
        else {
            velocity = vector3d(0, 0, 0);
        }
    }
    
    previous_position = position;
    position = new_position;
    
    // Here we can apply an autopilot to adjust the thrust, parachute and attitude
    if (autopilot_enabled) {
        orbital_injection_enabled = false;
        autopilot();
    }

    //Here I can apply an orbital injection autopilot to adjust the thrust, parachute and attitude.
    if (orbital_injection_enabled) {
        orbital_injection();
    }

    if (hohmann_enabled) {
        hohmann_transfer();
    }

    if (orbital_reentry_enabled) {
        orbital_reentry();
    }

    if (descending_sequence_enabled) {
        descending_sequence();
    }

    // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
    if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation(void)
// Lander pose initialization - selects one of 10 possible scenarios
{
    // The parameters to set are:
    // position - in Cartesian planetary coordinate system (m)
    // velocity - in Cartesian planetary coordinate system (m/s)
    // orientation - in lander coordinate system (xyz Euler angles, degrees)
    // delta_t - the simulation time step
    // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
    // scenario_description - a descriptive string for the help screen

    scenario_description[0] = "circular orbit";
    scenario_description[1] = "descent from 10km";
    scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
    scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
    scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
    scenario_description[5] = "descent from 200km";
    scenario_description[6] = "Areostationary orbit";
    scenario_description[7] = "Terrain at landing";
    scenario_description[8] = "";
    scenario_description[9] = "";

    switch (scenario) {

    case 0:
        // a circular equatorial orbit
        FUEL_RATE_AT_MAX_THRUST = 0.5;
        position = vector3d(1.2 * MARS_RADIUS, 0.0, 0.0);
        velocity = vector3d(0.0, -3247.087385863725, 0.0);
        orientation = vector3d(0.0, 90.0, 0.0);
        delta_t = 0.1;
        parachute_status = NOT_DEPLOYED;
        stabilized_attitude = false;
        autopilot_enabled = false;
        orbital_reentry_enabled = false;
        break;

    case 1:
        // a descent from rest at 10km altitude
        FUEL_RATE_AT_MAX_THRUST = 0.5;
        position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
        velocity = vector3d(0.0, 0.0, 0.0);
        orientation = vector3d(0.0, 0.0, 90.0);
        delta_t = 0.1;
        parachute_status = NOT_DEPLOYED;
        stabilized_attitude = true;
        autopilot_enabled = false;
        orbital_injection_enabled = false;
        orbital_reentry_enabled = false;
        break;

    case 2:
        // an elliptical polar orbit
        FUEL_RATE_AT_MAX_THRUST = 0.5;
        position = vector3d(0.0, 0.0, 1.2 * MARS_RADIUS);
        velocity = vector3d(3500.0, 0.0, 0.0);
        orientation = vector3d(0.0, 0.0, 90.0);
        delta_t = 0.1;
        parachute_status = NOT_DEPLOYED;
        stabilized_attitude = false;
        autopilot_enabled = false;
        orbital_reentry_enabled = false;
        break;

    case 3:
        // polar surface launch at escape velocity (but drag prevents escape)
        FUEL_RATE_AT_MAX_THRUST = 0.0;
        position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE / 2.0);
        velocity = vector3d(0.0, 0.0, 5027.0);
        orientation = vector3d(0.0, 0.0, 0.0);
        delta_t = 0.1;
        parachute_status = NOT_DEPLOYED;
        stabilized_attitude = false;
        autopilot_enabled = false;
        orbital_injection_enabled = false;
        orbital_reentry_enabled = false;

        break;

    case 4:
        // an elliptical orbit that clips the atmosphere each time round, losing energy
        FUEL_RATE_AT_MAX_THRUST = 0.5;
        position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
        velocity = vector3d(4000.0, 0.0, 0.0);
        orientation = vector3d(0.0, 90.0, 0.0);
        delta_t = 0.1;
        parachute_status = NOT_DEPLOYED;
        stabilized_attitude = false;
        autopilot_enabled = false;
        orbital_reentry_enabled = false;
        break;

    case 5:
        // a descent from rest at the edge of the exosphere
        FUEL_RATE_AT_MAX_THRUST = 0.5;
        position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
        velocity = vector3d(0.0, 0.0, 0.0);
        orientation = vector3d(0.0, 0.0, 90.0);
        delta_t = 0.1;
        parachute_status = NOT_DEPLOYED;
        stabilized_attitude = true;
        autopilot_enabled = false;
        orbital_injection_enabled = false;
        orbital_reentry_enabled = false;

        break;

    case 6:
        //Areostationary orbit - orbital time period = rotational period of Mars.
        FUEL_RATE_AT_MAX_THRUST = 0.5;
        position = vector3d(0.0, 20430493, 0.0);
        velocity = vector3d(-1447.96, 0.0, 0.0);
        orientation = vector3d(0.0, 90.0, 0.0);
        delta_t = 0.1;
        parachute_status = NOT_DEPLOYED;
        stabilized_attitude = false;
        autopilot_enabled = false;
        orbital_reentry_enabled = false;

        break;

    case 7:
        //Terrain at landing
        FUEL_RATE_AT_MAX_THRUST = 0.5;
        position = vector3d(0.0, MARS_RADIUS+1, 0.0);
        velocity = vector3d(0.0, 0.0, 0.0);
        orientation = vector3d(0.0, 0.0, -90.0);
        delta_t = 0.1;
        parachute_status = NOT_DEPLOYED;
        stabilized_attitude = false;
        autopilot_enabled = false;
        orbital_reentry_enabled = false;
        break;

    case 8:
        //tuning autopilot (same scenario settings as scenario 1)
        FUEL_RATE_AT_MAX_THRUST = 0.5;
        position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
        velocity = vector3d(0.0, 0.0, 0.0);
        orientation = vector3d(0.0, 0.0, 90.0);
        delta_t = 0.1;
        parachute_status = NOT_DEPLOYED;
        stabilized_attitude = true;
        autopilot_enabled = false;
        orbital_injection_enabled = false;
        orbital_reentry_enabled = false;
        break;

    case 9:
        break;

    }
}
