# 5. $\text{  }$ Practical fixed-time decentralized formation controller (PFxTDFC) <br/>

This markdown file **< README - Part 5 - 3.md >** includes comparative simulation studies in **Chapter 5.4** against some representitive distributed controllers published recently in IEEE Transactions. <br/> 

Please refer to **< README - Part 5 - 1.md >** for the Lyapunov-based theoretical instruction for the distributed controller design according to the stability proof in **Chapter 5.1**, and also for the implementation procedure of the proposed distributed control scheme in **Chapter 5.2**. <br/>

Refer to **< README - Part 5 - 2.md >** for the parameter settings and validation of the proposed distributed control scheme in **Chapter 5.3**. <br/> <br/>


## 5.4 $\text{  }$ Validation for comparison

The comparison approaches for distributed translational formation control are proposed in the references listed as follows. <br/>

[1]	L. Xu, Y. Wang, X. Wang, and C. Peng, “Distributed active disturbance rejection formation tracking control for quadrotor UAVs,” *IEEE Transactions on Cybernetics*, vol. 54, no. 8, pp. 4678–4689, Aug. 2024. <br/>
[2]	J. Wang, W. Wang, Y. Zou, and X. He, “Saturated distributed centroid tracking control of cluster formation and its application to VTOL AAVs,” *IEEE Transactions on Network Science and Engineering*, vol. 11, no. 6, pp. 5850–5862, Nov–Dec. 2024. <br/>
[3]	Y. Huang, X. Xu, Z. Meng, and J. Sun, “A smooth distributed formation control method for quadrotor UAVs under event-triggering mechanism and switching topologies,” *IEEE Transactions on Vehicular Technology*, vol. 74, no. 7, pp. 10081–10091, Jul. 2025. <br/>
[4]	Y. Hu, Z. Miao, Y. Wang, H. Tang, X. Wang, and W. He, “L1 adaptive control-based formation tracking of multiple quadrotors without linear velocity feedback under unknown disturbances,” *IEEE Transactions on Automation Science and Engineering*, vol. 22, pp. 5804–5815, Jul. 2024. <br/>

Refering to Ref.[1], the comparison controller is deployed on UAV formation systems, we can obtain the comparison simulation results as follows. <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Translational%20Control/Results%20IEEE_TASE_2025/3-D%20Trajectory%20IEEE_TASE_2025.png) <br/> <br/>

Fig.1 $\quad$ Three-Dimensional Trajectory Tracking Results for Ref.[1] <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Translational%20Control/Results%20IEEE_TASE_2025/Position%20Estimation%20IEEE_TASE_2025.png) <br/> <br/>

Fig.2 $\quad$ Position Estimation Results for Ref.[1] <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Translational%20Control/Results%20IEEE_TASE_2025/Velocity%20Estimation%20IEEE_TASE_2025.png) <br/> <br/>

Fig.3 $\quad$ Velocity Estimation Results for Ref.[1] <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Translational%20Control/Results%20IEEE_TASE_2025/Position%20Error%20IEEE_TASE_2025.png) <br/> <br/>

Fig.4 $\quad$ Position Tracking Error Results for Ref.[1] <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Translational%20Control/Results%20IEEE_TASE_2025/Velocity%20Error%20IEEE_TASE_2025.png) <br/> <br/>

Fig.5 $\quad$ Velocity Tracking Error Results for Ref.[1] <br/>


Refering to Ref.[2], the comparison controller is deployed on UAV formation systems, we can obtain the comparison simulation results as follows. <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Translational%20Control/Results%20IEEE_TCYB_2024/3-D%20Trajectory%20IEEE_TCYB_2024.png) <br/> <br/>

Fig.6 $\quad$ Three-Dimensional Trajectory Tracking Results for Ref.[2] <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Translational%20Control/Results%20IEEE_TCYB_2024/Position%20Error%20IEEE_TCYB_2024.png) <br/> <br/>

Fig.7 $\quad$ Position Tracking Error Results for Ref.[2] <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Translational%20Control/Results%20IEEE_TCYB_2024/Velocity%20Error%20IEEE_TCYB_2024.png) <br/> <br/>

Fig.8 $\quad$ Velocity Tracking Error Results for Ref.[2] <br/>


Refering to Ref.[3], the comparison controller is deployed on UAV formation systems, we can obtain the comparison simulation results as follows. <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Translational%20Control/Results%20IEEE_TNSE_2024/3-D%20Trajectory%20IEEE_TNSE_2024.png) <br/> <br/>

Fig.9 $\quad$ Three-Dimensional Trajectory Tracking Results for Ref.[3] <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Translational%20Control/Results%20IEEE_TNSE_2024/sliding%20surface%20ri%20IEEE_TNSE_2024.png) <br/> <br/>

Fig.10 $\quad$ Sliding Surface $r _i$ Results for Ref.[3] <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Translational%20Control/Results%20IEEE_TNSE_2024/sliding%20surface%20si%20IEEE_TNSE_2024.png) <br/> <br/>

Fig.11 $\quad$ Sliding Surface $s _i$ Results for Ref.[3] <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Translational%20Control/Results%20IEEE_TNSE_2024/Position%20Error%20IEEE_TNSE_2024.png) <br/> <br/>

Fig.12 $\quad$ Position Tracking Error Results for Ref.[3] <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Translational%20Control/Results%20IEEE_TNSE_2024/Velocity%20Error%20IEEE_TNSE_2024.png) <br/> <br/>

Fig.13 $\quad$ Velocity Tracking Error Results for Ref.[3] <br/>


Refering to Ref.[4], the comparison controller is deployed on UAV formation systems, we can obtain the comparison simulation results as follows. <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Translational%20Control/Results%20IEEE_TVT_2025/3-D%20Trajectory%20IEEE_TVT_2025.png) <br/> <br/>

Fig.14 $\quad$ Three-Dimensional Trajectory Tracking Results for Ref.[4] <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Translational%20Control/Results%20IEEE_TVT_2025/Event-Trigger%20Output%20Estimation%20IEEE_TVT_2025.png) <br/> <br/>

Fig.15 $\quad$ Event-Triggered Output Estimation Results for Ref.[4] <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Translational%20Control/Results%20IEEE_TVT_2025/Event-Trigger%20Position%20Estimation%20IEEE_TVT_2025.png) <br/> <br/>

Fig.16 $\quad$ Event-Triggered Position Estimation Results for Ref.[4] <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Translational%20Control/Results%20IEEE_TVT_2025/Position%20Error%20IEEE_TVT_2025.png) <br/> <br/>

Fig.17 $\quad$ Position Tracking Error Results for Ref.[4] <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Translational%20Control/Results%20IEEE_TVT_2025/Velocity%20Error%20IEEE_TVT_2025.png) <br/> <br/>

Fig.18 $\quad$ Velocity Tracking Error Results for Ref.[4] <br/>




