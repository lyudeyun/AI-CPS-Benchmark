# Benchmarks

AI-enabled CPS. 
The folder `model` stores the simulink models with different structures of neural networks (mainly Feedforward Neural Network).
The folder `nncontroller` stores the simulink blocks of trained neural network controllers.
The folder `nnconfig` stores the parameters of trained neural network controllers.


## System requirement


- Operating system: Linux or MacOS;

- Matlab (Simulink/Stateflow) version: >= 2020a. (Matlab license needed)

- Python version: >= 3.3

- MATLAB toolboxes dependency: 
  1. [Model Predictive Control Toolbox](https://www.mathworks.com/help/mpc/index.html) for ACC benchmark
  2. [Stateflow](https://www.mathworks.com/products/stateflow.html)
  3. [Deep Learning Toolbox](https://www.mathworks.com/products/deep-learning.html)

## Installation

- Clone the repository `git clone https://github.com/lyudeyun/Benchmarks.git`
- It is recommended to install the latest version of breach to handle the simulink models with reinforcement learning controller.
- Install [Breach](https://github.com/decyphir/breach)
  1. start matlab, set up a C/C++ compiler using the command `mex -setup`. (Refer to [here](https://www.mathworks.com/help/matlab/matlabexternal/changing-default-compiler.html) for more details.)
  2. navigate to `breach/` in Matlab commandline, and run `InstallBreach`
- Modify the seed of CMA-ES algorithm
  1. `vi /breach/Core/Algos/@BreachProblems/BreachProblems.m`
  2. in the function `setup_cmaes`, replace the line `solver_opt.Seed = 0` with `solver_opt.Seed = round(rem(now, 1)*1000000)`

 
 ## How to run the script 
 
 - For each benchmark, given a specification φ and an input signal u, XXX_falsification.m is used to validate if the model violates the specification φ.
 - Given a random input signal u, XXX_falsify.m aims to compute the robustness of the model under the specification φ.
 - Replace the path `/Users/ldy/git/breach/` and `/Users/ldy/git/Benchmarks/` with the path of breach and the repository on your own computer respectively.
 - Run the script.
