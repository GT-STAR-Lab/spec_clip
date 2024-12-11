# On the Surprising Effectiveness of Spectral Clipping in Learning Stable Linear Dynamics
- This is a repository containing the code for the paper ["On the Surprising Effectiveness of Spectral Clipping in Learning Stable Linear Dynamics"](https://arxiv.org/pdf/2412.01168).

- Some of the code is forked from ["Memory-Efficient Learning of Stable Linear Dynamical Systems for Prediction and Control"](https://github.com/giorgosmamakoukas/MemoryEfficientStableLDS)

- Code still under development.

## Code Instructions
- Please download the dataset to your local directory.
- Run ucsd.m with MATLAB. It will learn the linear dynamics of UCSD dataset with LS, SC, SOC, WLS and CG respectively. The script will automatically create a new directory \results_ucsd to save system matrices, training time and reconstruction error.
- Run dtdb.m with MATLAB. It will learn the linear dynamics of DTDB dataset with LS, SC and SOC respectively. The script will automatically create a new directory \results_dtdb to save system matrices, training time and reconstruction error.
- Run kodex.m with MATLAB. It will learn the linear dynamics of KODex dataset with LS, SC and SOC respectively. The script will automatically create a new directory \results_kodex to save system matrices, and training time.