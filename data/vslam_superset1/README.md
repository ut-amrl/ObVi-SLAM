# VSLAM Superset 1
Goes forward ten meters from the origin on the positive x-axis in steps of 0.25 meters on a sin wave of amplitude 1. Only points within 20 meters of the camera location are conssidered at each step.

## Landmark Noise Levels
1. Low Noise - Gaussian noise with sigma = 0.1m added to all landmarks
1. Medium Noise - Gaussian noise with sigma = 0.25m added to all landmarks
1. High Noise - Gaussian noise with sigma = 0.5m added to all landmarks

Note that the landmark groundtruth is always the value saved to file. Corrupted landmarks are not recorded, corrupted image measurements is actually what we are after. The landmark noise level corresponds to how corrupted the simulated image measurement will be.

## Density Levels
For each given noise level - there is also an associated density level - the actual number of features per image will vary but should should be roughly consistent. Note that only a fraction of the total landmarks are actually imaged.

1. Low Density - 1000 3D landmarks 
1. Medium Density - 2000 3D landmarks
1. High Density - 5000 3D landmarks