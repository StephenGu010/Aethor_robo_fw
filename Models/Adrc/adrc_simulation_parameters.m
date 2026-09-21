function parameters = adrc_simulation_parameters
%ADRC_SIMULATION_PARAMETERS Toy offline values; never hardware qualification.
% This file contains no measured motor parameters or hardware enable flags.
parameters.sampleTime = single(0.004);
parameters.duration = 6;
parameters.b0 = single(10);
parameters.wc = single(4);
parameters.wo = single(20);
parameters.damping = single(1);
parameters.referenceAmplitude = single(0.2);
parameters.referenceAcceleration = single(0.2);
parameters.torqueLimit = single(0.1);
parameters.torqueSlew = single(0.5);
parameters.torqueQuantum = single(0.001);
parameters.velocityQuantum = single(0.001);
parameters.noiseAmplitude = single(0.0002);
parameters.disturbanceAcceleration = single(0.3);
parameters.seed = 3519;
parameters.provenance = 'offline_synthetic_example_not_motor_measurement';
end
