# 2. $\text{  }$ Nonsingular Lie-algebra-based sliding mode attitude controller (NLSMAC)<br/>

**Note for content organization :** <br/> 

This markdown file **< README - Part 2 - 4.md >** includes comparative simulation studies in **Chapter 2.4** against some representitive methods published recently in IEEE Transactions. <br/> 

Please refer to **< README - Part 2 - 1.md >** for the Lyapunov-based theoretical instruction for rotational control scheme design according to the stability proof in **Chapter 2.1**. <br/> 

Refer to **< README - Part 2 - 2.md >** to check the implementation procedure of the proposed control scheme in **Chapter 2.2**. 

Refer to **< README - Part 2 - 3.md >** for the parameter settings and validation of the proposed method in **Chapter 2.3**. <br/>

## 2.4 $\text{  }$ Validation for comparison <br/>

The comparison methods for **rotational control** of UAV system is reproduced from references that listed as follows. <br/>

[1]	J. Zhang, Y. Liu, and T. Chai, “Singularity-free low-complexity fault-tolerant prescribed performance control for spacecraft attitude stabilization,” *IEEE Transactions on Automation Science and Engineering*, vol. 22, pp. 15408–15419, May 2025. <br/>
[2]	Y. Huang, B. Sun, Z. Meng, and J. Sun, “Adaptive formation tracking control of multiple vertical takeoff and landing UAVs with bearing-only measurements,” *IEEE Transactions on Cybernetics*, vol. 54, no. 6, pp. 3491–3501, Jun. 2024. <br/>
[3]	J. Lin, Z. Miao, Y. Wang, G. Hu, and R. Feirro, “Aggressive formation tracking for multiple quadrotors without velocity measurements over directed topologies,” *IEEE Transactions on Aerospace and Electronic Systems*, vol. 59, no. 5, pp. 5541–5553, Oct. 2023. <br/>
[4]	J. Lin, Z. Miao, Y. Wang, H. Wang, X. Wang, and R. Fierro, “Vision-based safety-critical landing control of quadrotors with external uncertainties and collision avoidance,” *IEEE Transactions on Control Systems Technology*, vol. 32, no. 4, pp. 1310–1322, Jul. 2024. <br/>

The simulation results are obtained for comparison from the reference paper Ref.[1] as follows. <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Rotational%20Control/Results%20IEEE_TAES_2023/Angular%20Velocity%20Error%20IEEE_TAES_2023.png) <br/> <br/>

Fig.1 Angular Velocity Tracking Errors for Comparison Methods in Ref.[1]. <br/>


![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Rotational%20Control/Results%20IEEE_TAES_2023/Exponential%20Coordinate%20Rotation%20Error%20IEEE_TAES_2023.png) <br/> <br/>

Fig.2 Exponential Coordinate Rotational Tracking Errors for Comparison Methods in Ref.[1]. <br/> <br/>


The simulation results are obtained for comparison from the reference paper Ref.[2] as follows. <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Rotational%20Control/Results%20IEEE_TASE_2025/Angular%20Velocity%20Error%20IEEE_TASE_2025.png) <br/> <br/>

Fig.3 Angular Velocity Tracking Errors for Comparison Methods in Ref.[2]. <br/>


![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Rotational%20Control/Results%20IEEE_TASE_2025/Exponential%20Coordinate%20Rotation%20Error%20IEEE_TASE_2025.png) <br/> <br/>

Fig.4 Exponential Coordinate Rotational Tracking Errors for Comparison Methods in Ref.[2]. <br/>


![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Rotational%20Control/Results%20IEEE_TASE_2025/Quaternion%20Error%20IEEE_TASE_2025.png) <br/> <br/>

Fig.5 Quaternion Errors for Comparison Methods in Ref.[2]. <br/>


![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Rotational%20Control/Results%20IEEE_TASE_2025/Auxiliary%20Rotational%20Error%20IEEE_TASE_2025.png) <br/> <br/>

Fig.6 Auxiliary Rotational Errors for Comparison Methods in Ref.[2]. <br/>




