# 3. $\text{  }$ Practical fixed-time distributed state observer (PFxTDSO) <br/>

This markdown file **< README - Part 3 - 3.md >** includes comparative simulation studies in **Chapter 3.4** against some representitive observers published recently in IEEE Transactions. <br/> 

Please refer to **< README - Part 3 - 1.md >** for the Lyapunov-based theoretical instruction for the distributed observer design according to the stability proof in **Chapter 3.1**, and also includes the implementation procedure of the proposed distributed observer scheme in **Chapter 3.2**. <br/>

Refer to **< README - Part 3 - 2.md >** for the parameter settings and validation of the proposed observer in **Chapter 3.3**. <br/> <br/>


## 3.4 $\text{  }$ Validation for comparison <br/>

Three comparison methods are given in the references listed as follows. <br/>

[1]	Q. Chen, Y. Zhao, G. Wen, G. Shi, and X. Yu, “Fixed-time coop erative tracking control for double-integrator multiagent systems: A time-based generator approach,” *IEEE Transactions on Cybernetics*, vol. 53, no. 9, pp. 5970–5983, Sep. 2023. <br/>
[2]	H. Gao, Y. Xia, K. Liu, J. Zhang, and B. Cui, “Resilient neuroadaptive distributed fixed-time attitude coordination control for multiple spacecraft,” *IEEE Transactions on Cybernetics*, vol. 54, no. 9, pp. 4973–4985, Sep. 2024. <br/>
[3]	G. Li, X. Wang, Z. Zuo, Y. Wu, and J. L¨u, “Distributed extended state observer-based formation control of flight vehicles subject to constraints on speed and acceleration,” *IEEE Transactions on Cybernetics*, vol. 55, no. 3, pp. 1250–1263, Mar. 2025. <br/> <br/>

**Please refer to < *Comparison Simulation Code/Comparison on Distributed State Observation/* > directory for the source code of four typical method that is published in IEEE Transactions and is used for comparison.** <br/> <br/>

The distributed observer proposed in Ref.[1] is employed and we can obtain the results as follows. <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Distributed%20Observation/Results%20IEEE_TCYB_2023/IEEE_TCYB_2023_Position_Observation.png) <br/> <br/>

Fig. 1 $\quad$ Distributed Position Observation Results <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Distributed%20Observation/Results%20IEEE_TCYB_2023/IEEE_TCYB_2023_Velocity_Observation.png) <br/> <br/>

Fig. 2 $\quad$ Distributed Velocity Observation Results <br/> <br/>

The distributed observer proposed in Ref.[2] is employed and we can obtain the results as follows. <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Distributed%20Observation/Results%20IEEE_TCYB_2024/IEEE_TCYB_2024_Position_Observation.png) <br/> <br/>

Fig. 3 $\quad$ Distributed Position Observation Results <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Distributed%20Observation/Results%20IEEE_TCYB_2024/IEEE_TCYB_2024_Velocity_Observation.png) <br/> <br/>

Fig. 4 $\quad$ Distributed Velocity Observation Results <br/> <br/>


The distributed observer proposed in Ref.[3] is employed and we can obtain the results as follows. <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Distributed%20Observation/Results%20IEEE_TCYB_2025/IEEE_TCYB_2025_Position_Observation.png) <br/> <br/>

Fig. 5 $\quad$ Distributed Position Observation Results <br/>

![image](https://github.com/zhangzyjoy/Code-for-CYB-E-2026-04-1374-2nd-revision-stage/blob/main/Simulation%20Results%20Pictures/Comparison%20Distributed%20Observation/Results%20IEEE_TCYB_2025/IEEE_TCYB_2025_Velocity_Observation.png) <br/> <br/>

Fig. 6 $\quad$ Distributed Velocity Observation Results <br/> <br/>

