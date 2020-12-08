This repo maintains the code for the NeurIPS 2020 paper:

Multi-Robot Collision Avoidance under Uncertainty with Probabilistic Safety Barrier Certificates  
Wenhao Luo, Wen Sun, and Ashish Kapoor  
**NeurIPS 2020**  
[[paper](https://proceedings.neurips.cc/paper/2020/file/03793ef7d06ffd63d34ade9d091f1ced-Paper.pdf)] [[video](http://www.cs.cmu.edu/~wenhaol/projects/NeurIPS20_PrSBC_video.mp4)]

# Get Started

The algorithm of Probablistic Safety Barrier Certificates (PrSBC) is implemented on a *modified* Robotarium Matlab Simulator originally provided by Georgia Tech. https://www.robotarium.gatech.edu/

The code can be run with MATLAB 2014b and higher with MATLAB's optimization toolbox function 'quadprog.'

1) Navigate to the root directory and run the "init.m" to initialize the addition of directories.

2) Go to path "..\examples" and run the demo codes

    a) "main_PrSBC.m" to get robots swapping positions using centralized PrSBC   
    b) "main_SBC.m" to get robots swapping positions using centralized safety barrier certificates from built-in Robotarium function    
    c) "main_PrSBC_dec.m" to get robots swapping positions using decentralized PrSBC

# Cite Us

If you find our work useful in your research, please consider citing:
```latex
@article{luo2020multi,
  title={Multi-Robot Collision Avoidance under Uncertainty with Probabilistic Safety Barrier Certificates},
  author={Luo, Wenhao and Sun, Wen and Kapoor, Ashish},
  journal={Advances in Neural Information Processing Systems},
  volume={33},
  year={2020}
}
```
    