# tree_diameter_estimator

## buildと実行
    ```
    #パッケージのるーとディレクトリにいる状態で
    cd build

    ##CMakeLists.txtのbuild
    cmake ..

    ##srcのbuild
    make

    ##生成した実行ファイルを動かす
    ./a.out
    ```
## Parameterについて
### VoxelGridDownsample
- leaf size (default: 0.01)

### StaticalOutlierRemoval
- 今の所なし

### Paththrough (z_axis)
- pt_min(default: 0.0)
- pt_max(default: 1.2)

### RANSAC Cylinder Segmentation
- normal_distance_weight(default: 0.1)
- max_iterations(default: 10000)
- distance_thres(default: 0.5)
- radius_min (default: 0.0)
- radius_max (default: 0.5)

