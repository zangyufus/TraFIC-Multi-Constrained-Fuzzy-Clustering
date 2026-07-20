# TraFIC: Multi-Constrained Fuzzy Clustering for Structure-Preserving Inconsistency Correction of Train-borne point clouds
This paper introduces TraFIC, a multi-constrained fuzzy clustering framework for structure-preserving inconsistency correction of train-borne railway point clouds. Railway MLS point clouds often suffer from position inconsistencies and non-rigid geometric distortions caused by GNSS degradation, platform attitude drift, and complex railway environments. To address these challenges, TraFIC integrates a railway geometry-aware covariance descriptor, a multi-constrained fuzzy clustering model, and a structure-preserving grid deformation field reconstruction strategy. The framework is specifically suitable for elongated and repetitive railway corridors, improving correspondence reliability while preserving slender structures such as rails and catenary lines. Extensive experiments on real-world railway scanning data and simulated deformation datasets demonstrate that TraFIC effectively restores relative geometric consistency. Overall, TraFIC provides a railway solution for reliable inconsistency correction and supports high-quality 3D railway infrastructure modeling.
## 🗂️ Dataset
Ⅰ 1-3 are dataset Ⅰ, Ⅱ 1-3 are dataset Ⅱ.
Dataset I (Fugro Dataset) was provided by Fugro and covers the Utrecht–Den Bosch railway corridor in the Netherlands. The data were collected using Fugro’s RILA mobile laser scanning system, equipped with a RIEGL laser scanner and an integrated GNSS/IMU navigation system. Dataset II (WHU-Railway3D Subset) is a publicly available railway point cloud dataset covering urban, rural, and plateau railway scenes. The data were collected using the Optech Lynx mobile mapping system, the HiScan-Z LiDAR sensor, and a rail-mounted mobile mapping system (rMMS).
<p align="center">
 <img width="764" height="661" alt="1" src="https://github.com/user-attachments/assets/0ccd8939-e0d6-4716-aa7b-fc8288fcaeaf" />
<img width="624" height="258" alt="2" src="https://github.com/user-attachments/assets/b1bab3fe-1b2f-4289-ad44-76f24ffb4e5c" />
</p>

## 🚀 Method
The proposed method focuses on relative geometric consistency correction between overlapping railway point clouds. First, a railway geometry-aware descriptor is introduced to characterize local geometric features. Second, a multi-constrained fuzzy clustering framework is developed to robustly estimate correspondences. Finally, a structure-preserving grid deformation field reconstruction method smoothly propagates sparse control-point deformations to the entire point cloud, thereby improving the geometric consistency of railway point clouds.The overview are as follow:
<p align="center">
<img width="1190" height="391" alt="3" src="https://github.com/user-attachments/assets/088feb4f-8865-4bf1-9250-fa78ea339a78" />
</p>

## 📊 Result
For each scene, the left side shows a comparison between the anchor point cloud and the floating point cloud, with red representing the floating points and green representing the anchor points. The right side shows a comparison between the anchor point cloud and the corrected point cloud, with green representing the anchor points and purple representing the corrected points. For each scene, two local zoomed-in views are also provided to observe the detail improvements.
<p align="center">
<img width="880" height="710" alt="4" src="https://github.com/user-attachments/assets/deec00b8-9fa8-439b-9216-583facdcf6ba" />
</p>
The quantitative evaluation results for Dataset I and Dataset II are listed in Tables Ⅱ and Ⅲ, respectively. Overall, the TraFIC method significantly reduces errors in the X, Y, and Z directions across all six scenes and effectively lowers the overall 3D distance error, indicating the TraFIC framework can reliably improve the geometric inconsistency of railway point clouds.
<p align="center">
<img width="1049" height="416" alt="5" src="https://github.com/user-attachments/assets/864e1811-c491-4793-a61a-21ea8355c194" />
</p>
Also, the special display section for the railway tracks is as follows,Green is the anchor point cloud, and purple is the corrected point cloud, used to check consistency.; see the full article for details.
<p align="center">
<img width="746" height="552" alt="6" src="https://github.com/user-attachments/assets/9c2a2268-abab-4c6a-96fd-09b53a6e5b48" />
</p>

## 💥 Comparison with Existing Methods
we selected two rigid registration methods, FGR and MuCoGraph, and two non-rigid registration methods, CPD and Fast_RNRR. The statistics of the three-axis errors as follow,See the paper for details.
<p align="center">
<img width="593" height="495" alt="7" src="https://github.com/user-attachments/assets/01f42f22-6562-4bea-ab54-6e4cea20fcad" />
</p>
Coming in future updates.
