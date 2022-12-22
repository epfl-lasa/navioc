This project is based on the software package CIOC by Sergey Levine, which is publicly available at the corresponding website https://graphics.stanford.edu/projects/cioc/. It is adopted in this repository's folder cioc.

### Datasets Pre-processing
The scripts processdiamor1.m and processdiamor2.m (in Preprocess/) load the original data from the datasets DIAMOR 1 and DIAMOR 2 and fit state-action trajectories to agents, considering only some time windows (as the datasets are very large). The script diamor1sample.m bundles trajectory segments of duration 4.8 s into multi-agent samples.

The script processeth.m (in Preprocess/) similarly loads the data from the dataset ETH, fits state-action trajectories to agents, and additionally cuts them in segments of duration 4.8 s and bundles them together in multi-agent samples.

### Learning Experiments

The script learningexperiments.m performs inverse reinforcement learning from the samples from DIAMOR 1. It then evaluates the learned model by re-optimizing samples from DIAMOR 1 and 2 and by making predictions for samples from ETH.
