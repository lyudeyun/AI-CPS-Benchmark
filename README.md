# AI-CPS-Benchmark

Artifact evaluation for ICSE 2022 SEIP submission "When Cyber-Physical Systems Meet AI: A Benchmark, an Evaluation, and a Way Forward" by Jiayang Song, Deyun Lyu, Zhenya Zhang, Zhijie Wang, Tianyi Zhang and Lei Ma.

## System requirement

- Operating system: Linux / MacOS / Windows;
- Matlab (Simulink/Stateflow) version: >= 2020a; (Matlab license needed)
- MATLAB toolboxes dependency
  1. [Simulink](https://www.mathworks.com/products/simulink.html)
  2. [Stateflow](https://www.mathworks.com/products/stateflow.html)
  3. [Model Predictive Control Toolbox](https://www.mathworks.com/products/model-predictive-control.html)
  4. [Deep Learning Toolbox](https://www.mathworks.com/products/deep-learning.html)
  5. [Reinforcement Learning Toolbox](https://www.mathworks.com/products/reinforcement-learning.html)
  6. [Automated Driving Toolbox](https://www.mathworks.com/products/automated-driving.html)
  
## Code Structure

- The directory `AI-CPS-Benchmark/` contains a `benchmarks/` sub-directory and a `tools/`. The former stores the 9 CPS models introduced in Benchmarks, and the latter involves two falsification tools, namely Breach and S-TaLiRo, used for RQ2: Falsification .
- The directory `benchmarks/` has 9 sub-directories, each storing one CPS model introduced in Benchmarks. Each model directory has a similar structure, and we take the directory `ACC/` as an example of illustration.
- `ACC/` involves the following sub-directories:
- `traditional/` stores the original ACC Simulink model with a traditional controller. 
- `DRL/` has three sub-directories: `train/`, `model/`, `agent/`. The folder `train/` stores the scripts for training DRL controllers,  `model/` stores the Simulink models with trained DRL controllers,  and `agent/` stores the trained agents (DRL controllers) based on different DRL algorithms.
- `RQ1/` stores the scripts for replicating the results of RQ1, where we compare the performances between AI-enabled CPS and traditional CPS;
- `RQ2/`  stores the scripts for replicating the results of RQ2, where we evaluate the effectiveness of different falsification approaches on AI-enabled CPS;
- `RQ3/` stores the scripts for replicating the results of RQ3, where we construct CPS with hybrid controllers and evaluate their performances using the methods in RQ1 and RQ2.

- The folder `tools/` includes two widely-used falsification tools, namely, Breach and S-TaLiRo. Specifically, we select Global Nelder-Mead (GNM) and CMAES for Breach, and Simulated Annealing (SA) and stochastic optimization with adaptive restart (SOAR) for S-TaLiRo. Details can be found [here](https://sites.google.com/view/ai-cps-benchmark/rq2-falsification).

## Installation

- Clone the repository `git clone https://github.com/lyudeyun/AI-CPS-Benchmarks.git`
- It is recommended to install the latest version of breach to handle the simulink models with reinforcement learning controller.
- Install [Breach](https://github.com/decyphir/breach)
  1. start matlab, set up a C/C++ compiler using the command `mex -setup`. (Refer to [here](https://www.mathworks.com/help/matlab/matlabexternal/changing-default-compiler.html) for more details.)
  2. navigate to `breach/` in Matlab commandline, and run `InstallBreach`
- Modify the configuration parameters of GNM algorithm
  1. `vi breach/Core/Algos/@BreachProblem/setup_global_nelder_mead.m`, then replace the line `'num_corners', min(2^dim_pb, 10*dim_pb)` with `'num_corners', 0` 
  2. `vi breach/Params/HaltonRefine.m`, then replace the line `seed = ones(dim_num,1)` with `seed = round(rem(now, 1)*1000000) * ones(dim_num,1)`
- Modify the seed of CMA-ES algorithm
  1. `vi /breach/Core/Algos/@BreachProblems/BreachProblems.m`
  2. in the function `setup_cmaes`, replace the line `solver_opt.Seed = 0` with `solver_opt.Seed = round(rem(now, 1)*1000000)`
- Install [S-TaLiRo](https://sites.google.com/a/asu.edu/s-taliro/s-taliro)
  1. If you have a previous version of the toolbox installed, remove it completely.
  2. Download the S-TaLiRo from [here](https://sites.google.com/a/asu.edu/s-taliro/s-taliro/download). 
  3. See the `readme.txt` file for any other Matlab packages that might be required.
  4. Run the `setup_staliro.m` file in the toolbox to setup the toolbox on your local machine.
  5. Type `help staliro` to get a detailed description of the toolbox and refer to the demos folder to run sample problems handled by the tool.
- Installation of SOAR algorithm, see [here](https://github.com/Lmathesen/S-TaLiRO-SOAR-Optimizers).

 ## How to run the script 
- Details can be found [here](https://sites.google.com/view/ai-cps-benchmark/replication-package)
