# MotionCor3
Anisotropic correction of beam induced motion for cryo-electron microscopy and cryo-electron tomography images.

MotionCor3, an improved implementation of MotionCor2 with addition of CTF (Contrast Transfer Function) estimation, is a multi-GPU accelerated software package that enables single-pixel level correction of anisotropic beam induced sample motion for cryo-electron microscopy and cryo-electron tomography images. The iterative, patch-based motion detection combined with spatial and temporal constraints and dose weighting provides robust and accurate correction. By refining the measurement of early motion, MotionCor3 further improves correction on tilted samples. The efficiency achieved by multi-GPU acceleration and parallelization enables correction to keep pace with automated data collection. The recent addition of a very robust GPU-accelerated CTF estimation makes MotionCor3 more versatile in cryoEM and cryoET processing pipeline.
![ReadmeImg](https://github.com/czimaginginstitute/MotionCor3/blob/master/ReadmeImg.png)

(**a**) Global motion in the direction perpendicular to the tilt axis (**b**) Residual local motion measured on a micrograph acquired at 0&deg; (**c**) Residual local motion measured on a micrograph acquired at 30&deg; (**d**) Residual local motion measured on a micrograph acquired at 60&deg;.


## Installation

MotionCor3 is developed on Linux platform equipped with at least one Nvidia GPU card. To compile from the source, follow the steps below:

1.	git clone https://github.com/czimaginginstitute/MotionCor3.git
2.	cd MotionCor3 
3.	make exe -f makefile11 [CUDAHOME=path/cuda-xx.x]

If the compute capability of GPUs is 5.x, use make instead of makefile11. If CUDAHOME is not provided, the default installation path of CUDA in makefile or makefile11 will be used.

## Code of Conduct

This project adheres to the Contributor Covenant [code of conduct](https://github.com/chanzuckerberg/.github/blob/master/CODE_OF_CONDUCT.md). By participating, you are expected to uphold this code. Please report unacceptable behavior to [opensource@chanzuckerberg.com](mailto:opensource@chanzuckerberg.com).

## Reporting Security Issues

If you believe you have found a security issue, please responsibly disclose by contacting us at [security@chanzuckerberg.com](mailto:security@chanzuckerberg.com).
