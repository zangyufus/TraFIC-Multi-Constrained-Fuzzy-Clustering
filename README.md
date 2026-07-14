# TraFIC: Multi-Constrained Fuzzy Clustering for Structure-Preserving Inconsistency Correction of Train-borne point clouds
This paper introduces TraFIC, a multi-constrained fuzzy clustering framework for structure-preserving inconsistency correction of train-borne railway point clouds. Railway MLS point clouds often suffer from position inconsistencies and non-rigid geometric distortions caused by GNSS degradation, platform attitude drift, and complex railway environments. To address these challenges, TraFIC integrates a railway geometry-aware covariance descriptor, a multi-constrained fuzzy clustering model, and a structure-preserving grid deformation field reconstruction strategy. The framework is specifically suitable for elongated and repetitive railway corridors, improving correspondence reliability while preserving slender structures such as rails and catenary lines. Extensive experiments on real-world railway scanning data and simulated deformation datasets demonstrate that TraFIC effectively restores relative geometric consistency. Overall, TraFIC provides a railway solution for reliable inconsistency correction and supports high-quality 3D railway infrastructure modeling.
## 🗂️ Dataset
Ⅰ 1-3 are dataset Ⅰ, Ⅱ 1-3 are dataset Ⅱ.
Dataset I (Fugro Dataset) was provided by Fugro and covers the Utrecht–Den Bosch railway corridor in the Netherlands. The data were collected using Fugro’s RILA mobile laser scanning system, equipped with a RIEGL laser scanner and an integrated GNSS/IMU navigation system. Dataset II (WHU-Railway3D Subset) is a publicly available railway point cloud dataset covering urban, rural, and plateau railway scenes. The data were collected using the Optech Lynx mobile mapping system, the HiScan-Z LiDAR sensor, and a rail-mounted mobile mapping system (rMMS).
<p align="center">
<img width="518" height="220" alt="image" src="https://github.com/user-attachments/assets/c09e85ee-318c-4f34-b38a-4990c817f7a6" />
<img width="518" height="219" alt="image" src="https://github.com/user-attachments/assets/7cecc1f7-42ad-495e-96b6-e68137c0869e" />
<img width="634" height="263" alt="image" src="https://github.com/user-attachments/assets/ad0afd37-65de-4dab-9db8-4884766a6f47" />
</p>

## 🚀 Method
The proposed method focuses on relative geometric consistency correction between overlapping railway point clouds. First, a railway geometry-aware descriptor is introduced to characterize local geometric features. Second, a multi-constrained fuzzy clustering framework is developed to robustly estimate correspondences. Finally, a structure-preserving grid deformation field reconstruction method smoothly propagates sparse control-point deformations to the entire point cloud, thereby improving the geometric consistency of railway point clouds.The overview are as follow:
<p align="center">
<img width="1080" height="344" alt="image" src="https://github.com/user-attachments/assets/411d7b65-04f0-459f-a20f-3ae066185581" />
</p>

## 📊 Result
For each scene, the left side shows a comparison between the anchor point cloud and the floating point cloud, with red representing the floating points and green representing the anchor points. The right side shows a comparison between the anchor point cloud and the corrected point cloud, with green representing the anchor points and purple representing the corrected points. For each scene, two local zoomed-in views are also provided to observe the detail improvements.
<p align="center">
<img width="865" height="669" alt="image" src="https://github.com/user-attachments/assets/879867b2-4483-4462-be54-44ff85587401" />
</p>
The quantitative evaluation results for Dataset I and Dataset II are listed in Tables Ⅱ and Ⅲ, respectively. Overall, the TraFIC method significantly reduces errors in the X, Y, and Z directions across all six scenes and effectively lowers the overall 3D distance error, indicating the TraFIC framework can reliably improve the geometric inconsistency of railway point clouds.
<p align="center">
<img width="865" height="340" alt="image" src="https://github.com/user-attachments/assets/d366e1b7-8d9f-4cb6-92f9-f68a83d40c5b" />
</p>
Also, the special display section for the railway tracks is as follows,Green is the anchor point cloud, and purple is the corrected point cloud, used to check consistency.; see the full article for details.
<p align="center">
<img width="799" height="584" alt="image" src="https://github.com/user-attachments/assets/3b9b0038-21e8-4ace-b48d-4dca6ab91026" />
</p>

## 💥 Comparison with Existing Methods
we selected two rigid registration methods, FGR and MuCoGraph, and two non-rigid registration methods, CPD and Fast_RNRR. The statistics of the three-axis errors as follow,See the paper for details.
<p align="center">
<img width="518" height="389" alt="image" src="https://github.com/user-attachments/assets/d19fa5b1-5f82-4dc9-a4bf-3aa16138034f" />
</p>
Coming in future updates.
