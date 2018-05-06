# mad_slam

go to /mad_slam/src/
run
python2 map_odom.py ../drone_data2/cmu_map.png 0.0323 64.5 51

Go to python/ folder, run test_vo_loop.py:

python3 test_vo_loop.py ../drone_data2/cmu_map.png ../drone_data2/images/ ../drone_data2/flight_vo_data.csv ../drone_data2/gt_traj.csv 0.0323 64.5 51 1 dlk_poses.csv jpg 1

Might get something like this warning when running:

>/Users/huntergoforth/.virtualenvs/cv3.6/lib/python3.6/site-packages/torch/serialization.py:316: SourceChangeWarning: source code of class 'DeepLKBatch.vgg16Conv' has changed. you can retrieve the original source code by accessing the object's source attribute or set torch.nn.Module.dump_patches =  True and use the patch tool to revert the changes.warnings.warn(msg, SourceChangeWarning)

After the warning, should output this:

>Loading pretrained network...done
>
>processing 0001/10  
>rotation:  0.087  
>trans_x: -5.493  
>trans_y:  2.402  
>scaling:  0.987  
>lk_scaling_factor:  1.323  
>x : 16.500, y : 15.240, h : -0.500, z : 11.200  
>xm: 16.478, ym: 15.495, hm: -0.413, zm: 11.345  
>processing 0002/10  
>rotation:  0.128  
>trans_x: -3.882  
>trans_y:  5.427  
>scaling:  1.011  
>lk_scaling_factor:  1.340  
>
>.  
>.  
>.  
>.  
>.  
>.  
>
>processing 0009/10  
>rotation: -0.021  
>trans_x: -1.187  
>trans_y: -1.101  
>scaling:  0.723  
>lk_scaling_factor:  1.425  
>x : 16.534, y : 15.780, h : -0.221, z : 12.064  
>xm: 16.473, ym: 15.822, hm: -0.243, zm: 16.691  
  
And then save to a file in the current directory called dlk_poses.csv  