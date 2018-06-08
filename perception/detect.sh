for path in '20171229_nanian_130255' '20180103_nanian_103247'
do
	for ((i=0; i<500; i ++))
	do
	#echo $i
	mdl /unsullied/sharefs/hqz/shared/hw/vehicle_detection_20180425/test_for_image_171108_pony.py -m /unsullied/sharefs/hqz/shared/hw/vehicle_detection_20180425/epoch_285.released -i /unsullied/sharefs/hqz/shared/hw/pony_data/perception_project/$path/select/GigECameraDeviceWideAngle/$i.jpg -o /unsullied/sharefs/hqz/shared/hw/pony_data/det/$path/$i"_"det.txt -s
	done
done



