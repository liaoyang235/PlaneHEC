*note: this repository is still under construction*  
PlaneHEC: Efficient Hand-Eye Calibration for Multi-view Robotic Arm via Any Point Cloud Plane Detection (ICRA 2025)

In order to calibrate, you have to collect `tf` data from your robot, and depth image from your rgbd camera. Data should be organized like the folder `data`. Example code will be given in `data_collection`, which was used with our Realsense and Flexiv robot.  

To calibrate your robot, run `main.m`, change the data path accordingly.  
To reproduce our numbers, run `main.m`, then run `evaluation/error_calc.m`.  

If this code or paper is helpful to you, please cite us:
```
@article{wang2025planehec,
  title={PlaneHEC: Efficient Hand-Eye Calibration for Multi-view Robotic Arm via Any Point Cloud Plane Detection},
  author={Wang, Ye and Jing, Haodong and Liao, Yang and Ma, Yongqiang and Zheng, Nanning},
  journal={arXiv preprint arXiv:2507.19851},
  year={2025}
}
```
