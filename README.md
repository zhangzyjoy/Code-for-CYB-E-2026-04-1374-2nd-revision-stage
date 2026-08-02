This Github repository demonstrates the proposed control approach for the formation control issue of uncrewed aerial vehicle (UAV) teams. **To address the reviewer's comments, the logical connection between theory and validation has been strengthened. Besides, the source code and implementation procedure of the approaches for comparison are given in this link.** <br/> <br/>

We upload **Matlab/Simulink** source code for **simulation** and **Python/ROS** code package for **experimental validation** in this webpage link to enhance the instructions on the relationship between theoretical contribution and further verification according to the reviewers' suggestions for our revised manuscript for **IEEE Transactions on Cybernetics** with a submission ID **CYB-E-2026-04-1374**. <br/> 

**Note :** Since **too many equations included in a single Markdown file may cause rendering errors or garbled output**, the theoretical design, implementation methodology, theory-guided parameter tuning, simulation experiments and comparative validation of the proposed control scheme are divided into multiple Markdown files to avoid this issue. Therefore, **each section < README - Part X .md > is divided into several subsections < README - Part X.X .md >**. <br/> <br/>

**The five main sections listed below are divided into several markdown files < README - Part 1.X .md > ~ < README - Part 5.X .md >, respectively.** <br/>

**The Appendix of significant theories and lemmas**, is utilized as an auxiliary instruction part in establishing the overall control framework, it is attached **at the back of each markdown file** for the reviewers' convenience to check the definitions and lemmas. <br/> <br/>

**This link includes five main sections. An additive appendix is given at the back of < README - Part X.1.md > to provide some essential Lemmas and fundamental theories that facilitate the design specifications.** Each of the first five items indicates a specific module of the proposed control scheme. <br/>
1. Fixed-time rotational disturbance observer (FxTDO). <br/>
2. Nonsingular Lie-algebra-based sliding mode attitude controller (NLSMAC). <br/>
3. Practical fixed-time distributed state observer (PFxTDSO). <br/>
4. Fixed-time translational disturbance observer (FxTDO). <br/>
5. Practical fixed-time decentralized formation controller (PFxTDFC). <br/>
Appendix : Significant lemmas and deductions. <br/><br/>

Each section contains four chapters, including : <br/>
<1>. Theories and design principles. <br/>
<2>. Implementation procedure. <br/>
<3>. Parameter settings and validation. <br/>
<4>. Validation for comparison. <br/> <br/>

**Some instructions on the chapters in each section :** <br/>
<1> It is worth mentioning that the first chapter entitled "theory and design principles" presents the **design principles** of the proposed controller and observer, which is developed by inverse design procedure under **Lyapunov-based stability analysis**. <br/>
<2> The second chapter entitled "implementation procedure" elaborates the overall framework of specific methods, including including detailed **mathematical formulations** and **overall implementation logic** of the control framework. <br/>
<3> The third chapter entitled "parameter settings and validation" discusses the **parameter settings** for simulation or experimental validation. Moreover, its **tuning principles** are given according to the **theories** proposed in the first chapter. <br/>
<4> The fourth chapter entitled "validation for comparison" illustrates the **comparative studies** on the **state-of-art methods published recently in IEEE Transactions** under the similar simulation or experiment settings, and thus the **performance metrices** of the proposed method are proved to be superior according to the comparison. <br/>

**Each section in this link follows the logic above, where the link theory-validation is enhanced.** <br/> <br/> 

**Note** : Source code and instructions of the overall control framework deposited during **the 1st revision with Revised & Resubmit decision** can be accessed at **https://github.com/zhangzyjoy/Code-for-manuscript-revision-stage.git** for your convenience. <br/>

**Please note that the validation part is different for the rotational and translational control subsystem, respectively. In our manuscript, the rotational control scheme is validated by numerical simulation only, however, the translational control scheme is validated by both numerical simulation and experimental verification.** <br/>

Specifically, the rotational control (NLSMAC) and rotational disturbance observation (Rotational FxTDO) are validated only through **numerical simulation**, whereas the translational control (PFxTDFC), translational distributed observer (PFxTDSO), and translational disturbance observer (Translational FxTDO) are validated through **both numerical simulation and real-world experiment**. <br/> <br/>

**The video for real-world experimental validation can be accessed at https://youtu.be/3SR10K3WDYw or https://www.bilibili.com/video/BV1T8XrBbEVf/** <br/> <br/>

**If there exist any question about this webpage or having interest on the source code of this paper, please do not hesitate to contact us and to discuss potential academic collaboration at any time by zyzhang9921@buaa.edu.cn, or zhaoyuzhang9921@gmail.com** <br/> <br/>

