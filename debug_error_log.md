logic@logicoma:/media/logic/USamsung/basalt_ros2/build$ ./basalt_calibrate \
  --dataset-path /media/logic/USamsung/oak_calibration/oak_calibration/oak_calibration_0.mcap \
  --dataset-type mcap \
  --aprilgrid /media/logic/USamsung/oak_calibration/aprilgrid.json \
  --result-path ~/oak_full_calib_result/ \
  --cam-types ds ds ds
Detected ROS2 bag format: MCAP (mcap)
Storage files: 1
imu_topic: /imu
mocap_topic: 
cam_topics: /colour/image_raw /left/image_raw /right/image_raw 

=== Camera Topic Mapping ===
  Camera 0: /colour/image_raw
  Camera 1: /left/image_raw
  Camera 2: /right/image_raw
===========================

Total messages processed: 11016
Time range: 1763729842162678522 to 1763729892566486267
Mocap-to-IMU offset: 12 ns
Total images indexed: 2022
Total IMU samples: 5018
Total mocap poses: 0
Loaded detected corners from: /home/logic/oak_full_calib_result/calib-cam_detected_corners.cereal
No pre-processed initial poses found
No calibration found
Started detecting corners
Done detecting corners. Saved them here: /home/logic/oak_full_calib_result/calib-cam_detected_corners.cereal
Started detecting corners

=== Started camera intrinsics initialization ===
Number of cameras: 3
Number of camera types specified: 3
Camera models: ds ds ds 
Total corner detections across all cameras: 2963
  Camera 0: 0 detections
  Camera 1: 0 detections
  Camera 2: 0 detections
Calling resetCalib...
resetCalib completed successfully

--- Initializing camera 0 (model: ds) ---
ERROR: Camera 0 has null image at timestamp 1763729842259453643
ERROR: Camera 0 has null image at timestamp 1763729842414387749
ERROR: Camera 0 has null image at timestamp 1763729842477201141
ERROR: Camera 0 has null image at timestamp 1763729842565827258
ERROR: Camera 0 has null image at timestamp 1763729842713204139
ERROR: Camera 0 has null image at timestamp 1763729842860468503
ERROR: Camera 0 has null image at timestamp 1763729843010402896
ERROR: Camera 0 has null image at timestamp 1763729843162910655
ERROR: Camera 0 has null image at timestamp 1763729843462133603
ERROR: Camera 0 has null image at timestamp 1763729843614665668
ERROR: Camera 0 has null image at timestamp 1763729843759567941
ERROR: Camera 0 has null image at timestamp 1763729843961421822
ERROR: Camera 0 has null image at timestamp 1763729844054830300
ERROR: Camera 0 has null image at timestamp 1763729844161597305
ERROR: Camera 0 has null image at timestamp 1763729844349479160
ERROR: Camera 0 has null image at timestamp 1763729844512502066
ERROR: Camera 0 has null image at timestamp 1763729844616768202
ERROR: Camera 0 has null image at timestamp 1763729844707225324
ERROR: Camera 0 has null image at timestamp 1763729844798513509
ERROR: Camera 0 has null image at timestamp 1763729844947824414
ERROR: Camera 0 has null image at timestamp 1763729845004684178
ERROR: Camera 0 has null image at timestamp 1763729845088208263
ERROR: Camera 0 has null image at timestamp 1763729845203714796
ERROR: Camera 0 has null image at timestamp 1763729845398904536
ERROR: Camera 0 has null image at timestamp 1763729845552970215
ERROR: Camera 0 has null image at timestamp 1763729845690834896
ERROR: Camera 0 has null image at timestamp 1763729845835012619
ERROR: Camera 0 has null image at timestamp 1763729845985980131
ERROR: Camera 0 has null image at timestamp 1763729846129749831
ERROR: Camera 0 has null image at timestamp 1763729846193879487
ERROR: Camera 0 has null image at timestamp 1763729846280552305
ERROR: Camera 0 has null image at timestamp 1763729846348411228
ERROR: Camera 0 has null image at timestamp 1763729846431052149
ERROR: Camera 0 has null image at timestamp 1763729846892404296
ERROR: Camera 0 has null image at timestamp 1763729847046758567
ERROR: Camera 0 has null image at timestamp 1763729847201076031
ERROR: Camera 0 has null image at timestamp 1763729847348383909
ERROR: Camera 0 has null image at timestamp 1763729847504641054
ERROR: Camera 0 has null image at timestamp 1763729847657417152
ERROR: Camera 0 has null image at timestamp 1763729847812682665
ERROR: Camera 0 has null image at timestamp 1763729847965022943
ERROR: Camera 0 has null image at timestamp 1763729848124091758
ERROR: Camera 0 has null image at timestamp 1763729848229425865
ERROR: Camera 0 has null image at timestamp 1763729848418594161
ERROR: Camera 0 has null image at timestamp 1763729848564000075
ERROR: Camera 0 has null image at timestamp 1763729848715127948
ERROR: Camera 0 has null image at timestamp 1763729848856458451
ERROR: Camera 0 has null image at timestamp 1763729849019239424
ERROR: Camera 0 has null image at timestamp 1763729849164259874
ERROR: Camera 0 has null image at timestamp 1763729849459925723
ERROR: Camera 0 has null image at timestamp 1763729849527199432
ERROR: Camera 0 has null image at timestamp 1763729849620327214
ERROR: Camera 0 has null image at timestamp 1763729849766846286
ERROR: Camera 0 has null image at timestamp 1763729850218539148
ERROR: Camera 0 has null image at timestamp 1763729850366834958
ERROR: Camera 0 has null image at timestamp 1763729850521928309
ERROR: Camera 0 has null image at timestamp 1763729850813100389
ERROR: Camera 0 has null image at timestamp 1763729850962050907
ERROR: Camera 0 has null image at timestamp 1763729851105413075
ERROR: Camera 0 has null image at timestamp 1763729851263910157
ERROR: Camera 0 has null image at timestamp 1763729851416181362
ERROR: Camera 0 has null image at timestamp 1763729851553062866
ERROR: Camera 0 has null image at timestamp 1763729851689730511
ERROR: Camera 0 has null image at timestamp 1763729851852810134
ERROR: Camera 0 has null image at timestamp 1763729852004584266
ERROR: Camera 0 has null image at timestamp 1763729852145779136
ERROR: Camera 0 has null image at timestamp 1763729852289677071
ERROR: Camera 0 has null image at timestamp 1763729852497656191
ERROR: Camera 0 has null image at timestamp 1763729852585065576
ERROR: Camera 0 has null image at timestamp 1763729852729259715
ERROR: Camera 0 has null image at timestamp 1763729852884647526
ERROR: Camera 0 has null image at timestamp 1763729853025129578
ERROR: Camera 0 has null image at timestamp 1763729853174367502
ERROR: Camera 0 has null image at timestamp 1763729853323675897
ERROR: Camera 0 has null image at timestamp 1763729853479451896
ERROR: Camera 0 has null image at timestamp 1763729853632100045
ERROR: Camera 0 has null image at timestamp 1763729853780049434
ERROR: Camera 0 has null image at timestamp 1763729853840047108
ERROR: Camera 0 has null image at timestamp 1763729853931673405
ERROR: Camera 0 has null image at timestamp 1763729854084511248
ERROR: Camera 0 has null image at timestamp 1763729854231533472
ERROR: Camera 0 has null image at timestamp 1763729854382634947
ERROR: Camera 0 has null image at timestamp 1763729854527505727
ERROR: Camera 0 has null image at timestamp 1763729854830936867
ERROR: Camera 0 has null image at timestamp 1763729854981060122
ERROR: Camera 0 has null image at timestamp 1763729855130838422
ERROR: Camera 0 has null image at timestamp 1763729855336369755
ERROR: Camera 0 has null image at timestamp 1763729855430772733
ERROR: Camera 0 has null image at timestamp 1763729855579526854
ERROR: Camera 0 has null image at timestamp 1763729855718806071
ERROR: Camera 0 has null image at timestamp 1763729855875521251
ERROR: Camera 0 has null image at timestamp 1763729856026795658
ERROR: Camera 0 has null image at timestamp 1763729856171818137
ERROR: Camera 0 has null image at timestamp 1763729856321689120
ERROR: Camera 0 has null image at timestamp 1763729856386242096
ERROR: Camera 0 has null image at timestamp 1763729856480962021
ERROR: Camera 0 has null image at timestamp 1763729856624763434
ERROR: Camera 0 has null image at timestamp 1763729856771854524
ERROR: Camera 0 has null image at timestamp 1763729856926290167
ERROR: Camera 0 has null image at timestamp 1763729857071804691
ERROR: Camera 0 has null image at timestamp 1763729857219870720
ERROR: Camera 0 has null image at timestamp 1763729857364550620
ERROR: Camera 0 has null image at timestamp 1763729857518803135
ERROR: Camera 0 has null image at timestamp 1763729857664773036
ERROR: Camera 0 has null image at timestamp 1763729857814237252
ERROR: Camera 0 has null image at timestamp 1763729857960174047
ERROR: Camera 0 has null image at timestamp 1763729858025191550
ERROR: Camera 0 has null image at timestamp 1763729858117736494
ERROR: Camera 0 has null image at timestamp 1763729858320594588
ERROR: Camera 0 has null image at timestamp 1763729858417244067
ERROR: Camera 0 has null image at timestamp 1763729858572396510
ERROR: Camera 0 has null image at timestamp 1763729858719214235
ERROR: Camera 0 has null image at timestamp 1763729859012926863
ERROR: Camera 0 has null image at timestamp 1763729859160225039
ERROR: Camera 0 has null image at timestamp 1763729859308034396
ERROR: Camera 0 has null image at timestamp 1763729859454281295
ERROR: Camera 0 has null image at timestamp 1763729859593124274
ERROR: Camera 0 has null image at timestamp 1763729859740792618
ERROR: Camera 0 has null image at timestamp 1763729859889338889
ERROR: Camera 0 has null image at timestamp 1763729859997316488
ERROR: Camera 0 has null image at timestamp 1763729860234205948
ERROR: Camera 0 has null image at timestamp 1763729860325722733
ERROR: Camera 0 has null image at timestamp 1763729860579604373
ERROR: Camera 0 has null image at timestamp 1763729860726555150
ERROR: Camera 0 has null image at timestamp 1763729860882599560
ERROR: Camera 0 has null image at timestamp 1763729861028435782
ERROR: Camera 0 has null image at timestamp 1763729861183126145
ERROR: Camera 0 has null image at timestamp 1763729861292371536
ERROR: Camera 0 has null image at timestamp 1763729861465319635
ERROR: Camera 0 has null image at timestamp 1763729861616900167
ERROR: Camera 0 has null image at timestamp 1763729861765916762
ERROR: Camera 0 has null image at timestamp 1763729861918788273
ERROR: Camera 0 has null image at timestamp 1763729862124132571
ERROR: Camera 0 has null image at timestamp 1763729862304505474
ERROR: Camera 0 has null image at timestamp 1763729862462033490
ERROR: Camera 0 has null image at timestamp 1763729862610492318
ERROR: Camera 0 has null image at timestamp 1763729862895256451
ERROR: Camera 0 has null image at timestamp 1763729863144219328
ERROR: Camera 0 has null image at timestamp 1763729863285199296
ERROR: Camera 0 has null image at timestamp 1763729863446634541
ERROR: Camera 0 has null image at timestamp 1763729863553093196
ERROR: Camera 0 has null image at timestamp 1763729863656583801
ERROR: Camera 0 has null image at timestamp 1763729863746016052
ERROR: Camera 0 has null image at timestamp 1763729863898069283
ERROR: Camera 0 has null image at timestamp 1763729864055256466
ERROR: Camera 0 has null image at timestamp 1763729864204607122
ERROR: Camera 0 has null image at timestamp 1763729864491655054
ERROR: Camera 0 has null image at timestamp 1763729864644684830
ERROR: Camera 0 has null image at timestamp 1763729864795072096
ERROR: Camera 0 has null image at timestamp 1763729864988578771
ERROR: Camera 0 has null image at timestamp 1763729865081690983
ERROR: Camera 0 has null image at timestamp 1763729865232780451
ERROR: Camera 0 has null image at timestamp 1763729865296624103
ERROR: Camera 0 has null image at timestamp 1763729865379508012
ERROR: Camera 0 has null image at timestamp 1763729865539190827
ERROR: Camera 0 has null image at timestamp 1763729865673390714
ERROR: Camera 0 has null image at timestamp 1763729865825894154
ERROR: Camera 0 has null image at timestamp 1763729865985709391
ERROR: Camera 0 has null image at timestamp 1763729866130827979
ERROR: Camera 0 has null image at timestamp 1763729866273152287
ERROR: Camera 0 has null image at timestamp 1763729866662237883
ERROR: Camera 0 has null image at timestamp 1763729866815792950
ERROR: Camera 0 has null image at timestamp 1763729866867439076
ERROR: Camera 0 has null image at timestamp 1763729866956363499
ERROR: Camera 0 has null image at timestamp 1763729867099498755
ERROR: Camera 0 has null image at timestamp 1763729867256236290
ERROR: Camera 0 has null image at timestamp 1763729867396958539
  Found 140 corners at frame 1014 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
ERROR: Camera 0 has null image at timestamp 1763729867555051588
ERROR: Camera 0 has null image at timestamp 1763729867699563171
ERROR: Camera 0 has null image at timestamp 1763729867851880131
ERROR: Camera 0 has null image at timestamp 1763729868002472038
ERROR: Camera 0 has null image at timestamp 1763729868138667971
ERROR: Camera 0 has null image at timestamp 1763729868197187210
ERROR: Camera 0 has null image at timestamp 1763729868287388295
ERROR: Camera 0 has null image at timestamp 1763729868724562425
ERROR: Camera 0 has null image at timestamp 1763729868792548328
ERROR: Camera 0 has null image at timestamp 1763729868886898787
ERROR: Camera 0 has null image at timestamp 1763729869088860104
ERROR: Camera 0 has null image at timestamp 1763729869177196799
ERROR: Camera 0 has null image at timestamp 1763729869319434433
ERROR: Camera 0 has null image at timestamp 1763729869427086218
ERROR: Camera 0 has null image at timestamp 1763729869525449509
ERROR: Camera 0 has null image at timestamp 1763729869619473942
ERROR: Camera 0 has null image at timestamp 1763729869766380024
ERROR: Camera 0 has null image at timestamp 1763729869916028911
ERROR: Camera 0 has null image at timestamp 1763729870062409704
ERROR: Camera 0 has null image at timestamp 1763729870351272233
ERROR: Camera 0 has null image at timestamp 1763729870507231230
ERROR: Camera 0 has null image at timestamp 1763729870649906850
ERROR: Camera 0 has null image at timestamp 1763729870793204004
ERROR: Camera 0 has null image at timestamp 1763729870954036856
ERROR: Camera 0 has null image at timestamp 1763729871096328969
ERROR: Camera 0 has null image at timestamp 1763729871244676402
ERROR: Camera 0 has null image at timestamp 1763729871385480998
ERROR: Camera 0 has null image at timestamp 1763729871545395485
ERROR: Camera 0 has null image at timestamp 1763729871692361005
ERROR: Camera 0 has null image at timestamp 1763729871928657086
ERROR: Camera 0 has null image at timestamp 1763729872036696361
ERROR: Camera 0 has null image at timestamp 1763729872127368329
ERROR: Camera 0 has null image at timestamp 1763729872290480580
ERROR: Camera 0 has null image at timestamp 1763729872349640071
ERROR: Camera 0 has null image at timestamp 1763729872432800770
ERROR: Camera 0 has null image at timestamp 1763729872583365092
ERROR: Camera 0 has null image at timestamp 1763729872724171155
ERROR: Camera 0 has null image at timestamp 1763729872878710872
ERROR: Camera 0 has null image at timestamp 1763729873022683758
ERROR: Camera 0 has null image at timestamp 1763729873173803821
ERROR: Camera 0 has null image at timestamp 1763729873282567928
ERROR: Camera 0 has null image at timestamp 1763729873480755977
ERROR: Camera 0 has null image at timestamp 1763729873638186009
  Found 144 corners at frame 1266 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
ERROR: Camera 0 has null image at timestamp 1763729873791241562
ERROR: Camera 0 has null image at timestamp 1763729873940551084
ERROR: Camera 0 has null image at timestamp 1763729873990723454
ERROR: Camera 0 has null image at timestamp 1763729874088696463
ERROR: Camera 0 has null image at timestamp 1763729874233794521
ERROR: Camera 0 has null image at timestamp 1763729874378342984
ERROR: Camera 0 has null image at timestamp 1763729874517785992
ERROR: Camera 0 has null image at timestamp 1763729874582143760
ERROR: Camera 0 has null image at timestamp 1763729874666612829
ERROR: Camera 0 has null image at timestamp 1763729874883989973
ERROR: Camera 0 has null image at timestamp 1763729874967577134
ERROR: Camera 0 has null image at timestamp 1763729875097028677
ERROR: Camera 0 has null image at timestamp 1763729875388080720
ERROR: Camera 0 has null image at timestamp 1763729875483038746
ERROR: Camera 0 has null image at timestamp 1763729875632289671
ERROR: Camera 0 has null image at timestamp 1763729875783787516
ERROR: Camera 0 has null image at timestamp 1763729875928757273
ERROR: Camera 0 has null image at timestamp 1763729876085151743
ERROR: Camera 0 has null image at timestamp 1763729876233838826
ERROR: Camera 0 has null image at timestamp 1763729876373576781
ERROR: Camera 0 has null image at timestamp 1763729876521695411
ERROR: Camera 0 has null image at timestamp 1763729876668537101
ERROR: Camera 0 has null image at timestamp 1763729876819300687
ERROR: Camera 0 has null image at timestamp 1763729877068973389
ERROR: Camera 0 has null image at timestamp 1763729877220596464
ERROR: Camera 0 has null image at timestamp 1763729877369928128
ERROR: Camera 0 has null image at timestamp 1763729877517289451
ERROR: Camera 0 has null image at timestamp 1763729877572254240
ERROR: Camera 0 has null image at timestamp 1763729877655676154
ERROR: Camera 0 has null image at timestamp 1763729877818420264
ERROR: Camera 0 has null image at timestamp 1763729877958937668
ERROR: Camera 0 has null image at timestamp 1763729878097769272
ERROR: Camera 0 has null image at timestamp 1763729878251259460
ERROR: Camera 0 has null image at timestamp 1763729878395242895
ERROR: Camera 0 has null image at timestamp 1763729878547993376
ERROR: Camera 0 has null image at timestamp 1763729878707188624
ERROR: Camera 0 has null image at timestamp 1763729878855741818
ERROR: Camera 0 has null image at timestamp 1763729878998284880
ERROR: Camera 0 has null image at timestamp 1763729879147297153
ERROR: Camera 0 has null image at timestamp 1763729879291217101
ERROR: Camera 0 has null image at timestamp 1763729879444209798
ERROR: Camera 0 has null image at timestamp 1763729879554927766
ERROR: Camera 0 has null image at timestamp 1763729879695832800
ERROR: Camera 0 has null image at timestamp 1763729879789659300
ERROR: Camera 0 has null image at timestamp 1763729879877944105
  Found 48 corners at frame 1518 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
ERROR: Camera 0 has null image at timestamp 1763729880026056660
ERROR: Camera 0 has null image at timestamp 1763729880181186345
ERROR: Camera 0 has null image at timestamp 1763729880237879401
ERROR: Camera 0 has null image at timestamp 1763729880329462753
ERROR: Camera 0 has null image at timestamp 1763729880481550913
ERROR: Camera 0 has null image at timestamp 1763729880629962886
ERROR: Camera 0 has null image at timestamp 1763729880782122776
ERROR: Camera 0 has null image at timestamp 1763729880921757086
ERROR: Camera 0 has null image at timestamp 1763729881077801785
ERROR: Camera 0 has null image at timestamp 1763729881138099998
ERROR: Camera 0 has null image at timestamp 1763729881237406034
ERROR: Camera 0 has null image at timestamp 1763729881459507575
ERROR: Camera 0 has null image at timestamp 1763729881556140229
  Found 144 corners at frame 1584 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
ERROR: Camera 0 has null image at timestamp 1763729881708386305
ERROR: Camera 0 has null image at timestamp 1763729881855143557
ERROR: Camera 0 has null image at timestamp 1763729881914925351
ERROR: Camera 0 has null image at timestamp 1763729882005289733
ERROR: Camera 0 has null image at timestamp 1763729882147645687
ERROR: Camera 0 has null image at timestamp 1763729882288821504
  Found 144 corners at frame 1614 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
ERROR: Camera 0 has null image at timestamp 1763729882455026688
ERROR: Camera 0 has null image at timestamp 1763729882607335484
  Found 144 corners at frame 1626 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
ERROR: Camera 0 has null image at timestamp 1763729882750859703
ERROR: Camera 0 has null image at timestamp 1763729882899807652
ERROR: Camera 0 has null image at timestamp 1763729882948455000
ERROR: Camera 0 has null image at timestamp 1763729883046696838
  Found 127 corners at frame 1644 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
ERROR: Camera 0 has null image at timestamp 1763729883196661283
ERROR: Camera 0 has null image at timestamp 1763729883333678926
ERROR: Camera 0 has null image at timestamp 1763729883488410367
ERROR: Camera 0 has null image at timestamp 1763729883627760138
ERROR: Camera 0 has null image at timestamp 1763729883786818495
ERROR: Camera 0 has null image at timestamp 1763729883927671707
ERROR: Camera 0 has null image at timestamp 1763729883994601936
ERROR: Camera 0 has null image at timestamp 1763729884084101592
ERROR: Camera 0 has null image at timestamp 1763729884233300767
ERROR: Camera 0 has null image at timestamp 1763729884380965212
ERROR: Camera 0 has null image at timestamp 1763729884433676936
ERROR: Camera 0 has null image at timestamp 1763729884527636559
ERROR: Camera 0 has null image at timestamp 1763729884676017802
  Found 144 corners at frame 1710 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
ERROR: Camera 0 has null image at timestamp 1763729884826837336
ERROR: Camera 0 has null image at timestamp 1763729884966351169
ERROR: Camera 0 has null image at timestamp 1763729885120036502
ERROR: Camera 0 has null image at timestamp 1763729885415450881
  Found 144 corners at frame 1740 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
  Found 144 corners at frame 1743 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
ERROR: Camera 0 has null image at timestamp 1763729885629010132
ERROR: Camera 0 has null image at timestamp 1763729885818194165
  Found 144 corners at frame 1755 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
ERROR: Camera 0 has null image at timestamp 1763729885962437698
ERROR: Camera 0 has null image at timestamp 1763729886121568414
  Found 144 corners at frame 1770 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
  Found 144 corners at frame 1773 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
ERROR: Camera 0 has null image at timestamp 1763729886417223961
ERROR: Camera 0 has null image at timestamp 1763729886477912668
ERROR: Camera 0 has null image at timestamp 1763729886563908160
ERROR: Camera 0 has null image at timestamp 1763729886779118781
ERROR: Camera 0 has null image at timestamp 1763729886866682039
ERROR: Camera 0 has null image at timestamp 1763729887013467789
  Found 144 corners at frame 1803 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
ERROR: Camera 0 has null image at timestamp 1763729887160939539
ERROR: Camera 0 has null image at timestamp 1763729887370945108
ERROR: Camera 0 has null image at timestamp 1763729887454103855
ERROR: Camera 0 has null image at timestamp 1763729887720009973
  Found 144 corners at frame 1833 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
  Found 144 corners at frame 1836 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
ERROR: Camera 0 has null image at timestamp 1763729887978505046
ERROR: Camera 0 has null image at timestamp 1763729888162042970
ERROR: Camera 0 has null image at timestamp 1763729888274099342
ERROR: Camera 0 has null image at timestamp 1763729888366348440
ERROR: Camera 0 has null image at timestamp 1763729888462863830
ERROR: Camera 0 has null image at timestamp 1763729888615344024
ERROR: Camera 0 has null image at timestamp 1763729888766014235
ERROR: Camera 0 has null image at timestamp 1763729888920063871
ERROR: Camera 0 has null image at timestamp 1763729889285175175
  Found 0 corners at frame 1896 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
ERROR: Camera 0 has null image at timestamp 1763729889484669229
  Found 0 corners at frame 1902 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
ERROR: Camera 0 has null image at timestamp 1763729889672403323
ERROR: Camera 0 has null image at timestamp 1763729889736573846
ERROR: Camera 0 has null image at timestamp 1763729889823046715
ERROR: Camera 0 has null image at timestamp 1763729889986514476
ERROR: Camera 0 has null image at timestamp 1763729890146144222
ERROR: Camera 0 has null image at timestamp 1763729890294473156
ERROR: Camera 0 has null image at timestamp 1763729890512112990
ERROR: Camera 0 has null image at timestamp 1763729890599903449
ERROR: Camera 0 has null image at timestamp 1763729890908512159
ERROR: Camera 0 has null image at timestamp 1763729891055583639
  Found 0 corners at frame 1962 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
  Found 0 corners at frame 1965 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
ERROR: Camera 0 has null image at timestamp 1763729891264317160
ERROR: Camera 0 has null image at timestamp 1763729891353009642
ERROR: Camera 0 has null image at timestamp 1763729891560513227
ERROR: Camera 0 has null image at timestamp 1763729892015156877
ERROR: Camera 0 has null image at timestamp 1763729892100263059
ERROR: Camera 0 has null image at timestamp 1763729892315188655
ERROR: Camera 0 has null image at timestamp 1763729892411534746
ERROR: Camera 0 has null image at timestamp 1763729892552717970
  ⚠ Camera 0 not initialized yet, will try pinhole fallback

--- Initializing camera 1 (model: ds) ---
ERROR: Camera 1 has null image at timestamp 1763729842162678522
  Found 144 corners at frame 3 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729842318630659
  Found 144 corners at frame 9 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729842619927874
ERROR: Camera 1 has null image at timestamp 1763729842772134190
ERROR: Camera 1 has null image at timestamp 1763729842914143704
ERROR: Camera 1 has null image at timestamp 1763729843073335022
ERROR: Camera 1 has null image at timestamp 1763729843226434002
ERROR: Camera 1 has null image at timestamp 1763729843272673017
ERROR: Camera 1 has null image at timestamp 1763729843362045826
ERROR: Camera 1 has null image at timestamp 1763729843516390387
ERROR: Camera 1 has null image at timestamp 1763729843667779960
ERROR: Camera 1 has null image at timestamp 1763729843817548406
ERROR: Camera 1 has null image at timestamp 1763729843866690237
ERROR: Camera 1 has null image at timestamp 1763729844116095902
ERROR: Camera 1 has null image at timestamp 1763729844247508974
ERROR: Camera 1 has null image at timestamp 1763729844399193843
ERROR: Camera 1 has null image at timestamp 1763729844552953958
ERROR: Camera 1 has null image at timestamp 1763729844855018052
ERROR: Camera 1 has null image at timestamp 1763729845152844699
ERROR: Camera 1 has null image at timestamp 1763729845299029431
ERROR: Camera 1 has null image at timestamp 1763729845441688976
ERROR: Camera 1 has null image at timestamp 1763729845592686730
ERROR: Camera 1 has null image at timestamp 1763729845741071936
ERROR: Camera 1 has null image at timestamp 1763729845894684194
ERROR: Camera 1 has null image at timestamp 1763729846035514201
ERROR: Camera 1 has null image at timestamp 1763729846492721653
ERROR: Camera 1 has null image at timestamp 1763729846535205279
ERROR: Camera 1 has null image at timestamp 1763729846640160766
ERROR: Camera 1 has null image at timestamp 1763729846699488085
ERROR: Camera 1 has null image at timestamp 1763729846794080544
ERROR: Camera 1 has null image at timestamp 1763729846956862772
ERROR: Camera 1 has null image at timestamp 1763729847110572043
ERROR: Camera 1 has null image at timestamp 1763729847252337458
ERROR: Camera 1 has null image at timestamp 1763729847412680420
ERROR: Camera 1 has null image at timestamp 1763729847566634630
ERROR: Camera 1 has null image at timestamp 1763729847717849737
ERROR: Camera 1 has null image at timestamp 1763729847868324438
ERROR: Camera 1 has null image at timestamp 1763729848023946360
ERROR: Camera 1 has null image at timestamp 1763729848169796685
ERROR: Camera 1 has null image at timestamp 1763729848319739531
ERROR: Camera 1 has null image at timestamp 1763729848476915391
ERROR: Camera 1 has null image at timestamp 1763729848613104540
ERROR: Camera 1 has null image at timestamp 1763729848759084773
ERROR: Camera 1 has null image at timestamp 1763729848924685657
ERROR: Camera 1 has null image at timestamp 1763729849072691199
ERROR: Camera 1 has null image at timestamp 1763729849212549411
ERROR: Camera 1 has null image at timestamp 1763729849273403499
ERROR: Camera 1 has null image at timestamp 1763729849361184238
ERROR: Camera 1 has null image at timestamp 1763729849679695740
ERROR: Camera 1 has null image at timestamp 1763729849816788589
ERROR: Camera 1 has null image at timestamp 1763729849884543307
ERROR: Camera 1 has null image at timestamp 1763729849976025098
ERROR: Camera 1 has null image at timestamp 1763729850031915582
ERROR: Camera 1 has null image at timestamp 1763729850121685031
ERROR: Camera 1 has null image at timestamp 1763729850277581368
ERROR: Camera 1 has null image at timestamp 1763729850427089443
ERROR: Camera 1 has null image at timestamp 1763729850563027787
ERROR: Camera 1 has null image at timestamp 1763729850629854222
ERROR: Camera 1 has null image at timestamp 1763729850719378452
ERROR: Camera 1 has null image at timestamp 1763729850873031362
ERROR: Camera 1 has null image at timestamp 1763729851024313240
ERROR: Camera 1 has null image at timestamp 1763729851163170952
ERROR: Camera 1 has null image at timestamp 1763729851316798299
ERROR: Camera 1 has null image at timestamp 1763729851466307702
ERROR: Camera 1 has null image at timestamp 1763729851594498429
ERROR: Camera 1 has null image at timestamp 1763729851759430149
ERROR: Camera 1 has null image at timestamp 1763729851900770850
ERROR: Camera 1 has null image at timestamp 1763729852056430281
ERROR: Camera 1 has null image at timestamp 1763729852191854999
ERROR: Camera 1 has null image at timestamp 1763729852341339817
ERROR: Camera 1 has null image at timestamp 1763729852403805394
ERROR: Camera 1 has null image at timestamp 1763729852644596907
ERROR: Camera 1 has null image at timestamp 1763729852785350020
ERROR: Camera 1 has null image at timestamp 1763729852927423096
ERROR: Camera 1 has null image at timestamp 1763729853084715317
ERROR: Camera 1 has null image at timestamp 1763729853218538955
ERROR: Camera 1 has null image at timestamp 1763729853379750697
ERROR: Camera 1 has null image at timestamp 1763729853536855879
ERROR: Camera 1 has null image at timestamp 1763729853687642711
ERROR: Camera 1 has null image at timestamp 1763729853989623840
ERROR: Camera 1 has null image at timestamp 1763729854134596520
ERROR: Camera 1 has null image at timestamp 1763729854294778706
ERROR: Camera 1 has null image at timestamp 1763729854443056009
ERROR: Camera 1 has null image at timestamp 1763729854579200112
ERROR: Camera 1 has null image at timestamp 1763729854645834550
ERROR: Camera 1 has null image at timestamp 1763729854739304354
ERROR: Camera 1 has null image at timestamp 1763729854887009154
ERROR: Camera 1 has null image at timestamp 1763729855037929456
ERROR: Camera 1 has null image at timestamp 1763729855192328920
ERROR: Camera 1 has null image at timestamp 1763729855235394130
ERROR: Camera 1 has null image at timestamp 1763729855486224045
ERROR: Camera 1 has null image at timestamp 1763729855619828725
ERROR: Camera 1 has null image at timestamp 1763729855788182895
ERROR: Camera 1 has null image at timestamp 1763729855933131410
ERROR: Camera 1 has null image at timestamp 1763729856082871088
ERROR: Camera 1 has null image at timestamp 1763729856232103146
ERROR: Camera 1 has null image at timestamp 1763729856530269731
ERROR: Camera 1 has null image at timestamp 1763729856683776881
ERROR: Camera 1 has null image at timestamp 1763729856830917419
ERROR: Camera 1 has null image at timestamp 1763729856976811540
ERROR: Camera 1 has null image at timestamp 1763729857071804691
ERROR: Camera 1 has null image at timestamp 1763729857122555498
ERROR: Camera 1 has null image at timestamp 1763729857272655915
ERROR: Camera 1 has null image at timestamp 1763729857428661141
ERROR: Camera 1 has null image at timestamp 1763729857566195470
ERROR: Camera 1 has null image at timestamp 1763729857727631969
ERROR: Camera 1 has null image at timestamp 1763729857864238993
ERROR: Camera 1 has null image at timestamp 1763729858177969050
ERROR: Camera 1 has null image at timestamp 1763729858223991205
ERROR: Camera 1 has null image at timestamp 1763729858472603815
ERROR: Camera 1 has null image at timestamp 1763729858629124204
ERROR: Camera 1 has null image at timestamp 1763729858769257602
ERROR: Camera 1 has null image at timestamp 1763729858830193897
ERROR: Camera 1 has null image at timestamp 1763729858918644433
ERROR: Camera 1 has null image at timestamp 1763729859069647014
ERROR: Camera 1 has null image at timestamp 1763729859203206019
ERROR: Camera 1 has null image at timestamp 1763729859368401540
ERROR: Camera 1 has null image at timestamp 1763729859497241950
ERROR: Camera 1 has null image at timestamp 1763729859656423428
ERROR: Camera 1 has null image at timestamp 1763729859798353958
ERROR: Camera 1 has null image at timestamp 1763729859942021903
ERROR: Camera 1 has null image at timestamp 1763729860091072860
ERROR: Camera 1 has null image at timestamp 1763729860137761318
ERROR: Camera 1 has null image at timestamp 1763729860386393486
ERROR: Camera 1 has null image at timestamp 1763729860490522385
ERROR: Camera 1 has null image at timestamp 1763729860641762572
ERROR: Camera 1 has null image at timestamp 1763729860784190594
ERROR: Camera 1 has null image at timestamp 1763729860936115522
ERROR: Camera 1 has null image at timestamp 1763729861086005993
ERROR: Camera 1 has null image at timestamp 1763729861245507912
ERROR: Camera 1 has null image at timestamp 1763729861372928484
ERROR: Camera 1 has null image at timestamp 1763729861530189909
ERROR: Camera 1 has null image at timestamp 1763729861675600089
ERROR: Camera 1 has null image at timestamp 1763729861830046211
ERROR: Camera 1 has null image at timestamp 1763729861963432425
ERROR: Camera 1 has null image at timestamp 1763729862054035175
ERROR: Camera 1 has null image at timestamp 1763729862204032716
ERROR: Camera 1 has null image at timestamp 1763729862358220768
ERROR: Camera 1 has null image at timestamp 1763729862523443531
ERROR: Camera 1 has null image at timestamp 1763729862664583858
ERROR: Camera 1 has null image at timestamp 1763729862756012921
ERROR: Camera 1 has null image at timestamp 1763729862802451480
ERROR: Camera 1 has null image at timestamp 1763729862950192183
ERROR: Camera 1 has null image at timestamp 1763729863053292436
ERROR: Camera 1 has null image at timestamp 1763729863198973398
ERROR: Camera 1 has null image at timestamp 1763729863354805067
ERROR: Camera 1 has null image at timestamp 1763729863501430448
ERROR: Camera 1 has null image at timestamp 1763729863809911039
ERROR: Camera 1 has null image at timestamp 1763729863953117253
ERROR: Camera 1 has null image at timestamp 1763729864106218130
ERROR: Camera 1 has null image at timestamp 1763729864253924471
ERROR: Camera 1 has null image at timestamp 1763729864315465468
ERROR: Camera 1 has null image at timestamp 1763729864405596779
ERROR: Camera 1 has null image at timestamp 1763729864550438019
ERROR: Camera 1 has null image at timestamp 1763729864687585422
ERROR: Camera 1 has null image at timestamp 1763729864850285175
ERROR: Camera 1 has null image at timestamp 1763729864896517838
ERROR: Camera 1 has null image at timestamp 1763729865138525049
ERROR: Camera 1 has null image at timestamp 1763729865446668649
ERROR: Camera 1 has null image at timestamp 1763729865588450626
ERROR: Camera 1 has null image at timestamp 1763729865725200342
ERROR: Camera 1 has null image at timestamp 1763729865877864421
ERROR: Camera 1 has null image at timestamp 1763729866033567301
ERROR: Camera 1 has null image at timestamp 1763729866188105967
ERROR: Camera 1 has null image at timestamp 1763729866327039746
ERROR: Camera 1 has null image at timestamp 1763729866385401977
ERROR: Camera 1 has null image at timestamp 1763729866486038867
ERROR: Camera 1 has null image at timestamp 1763729866577787464
ERROR: Camera 1 has null image at timestamp 1763729866725189849
ERROR: Camera 1 has null image at timestamp 1763729867019284525
ERROR: Camera 1 has null image at timestamp 1763729867156902461
ERROR: Camera 1 has null image at timestamp 1763729867300792511
  Found 144 corners at frame 1011 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729867461902359
  Found 144 corners at frame 1017 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729867608708006
  Found 144 corners at frame 1023 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729867756431111
ERROR: Camera 1 has null image at timestamp 1763729867904760593
ERROR: Camera 1 has null image at timestamp 1763729868055728047
ERROR: Camera 1 has null image at timestamp 1763729868345708412
ERROR: Camera 1 has null image at timestamp 1763729868392598577
ERROR: Camera 1 has null image at timestamp 1763729868492165891
ERROR: Camera 1 has null image at timestamp 1763729868537284698
ERROR: Camera 1 has null image at timestamp 1763729868629927355
ERROR: Camera 1 has null image at timestamp 1763729868941463933
ERROR: Camera 1 has null image at timestamp 1763729868989751727
ERROR: Camera 1 has null image at timestamp 1763729869232339547
ERROR: Camera 1 has null image at timestamp 1763729869382552277
ERROR: Camera 1 has null image at timestamp 1763729869679609279
ERROR: Camera 1 has null image at timestamp 1763729869819633378
ERROR: Camera 1 has null image at timestamp 1763729869963400086
ERROR: Camera 1 has null image at timestamp 1763729870123061740
ERROR: Camera 1 has null image at timestamp 1763729870170005895
ERROR: Camera 1 has null image at timestamp 1763729870255395703
ERROR: Camera 1 has null image at timestamp 1763729870411686245
ERROR: Camera 1 has null image at timestamp 1763729870552970522
ERROR: Camera 1 has null image at timestamp 1763729870695733237
ERROR: Camera 1 has null image at timestamp 1763729870860289910
ERROR: Camera 1 has null image at timestamp 1763729871005868556
ERROR: Camera 1 has null image at timestamp 1763729871154940960
ERROR: Camera 1 has null image at timestamp 1763729871291065724
ERROR: Camera 1 has null image at timestamp 1763729871453926539
ERROR: Camera 1 has null image at timestamp 1763729871595063169
ERROR: Camera 1 has null image at timestamp 1763729871749423808
ERROR: Camera 1 has null image at timestamp 1763729871842874410
ERROR: Camera 1 has null image at timestamp 1763729871990349505
ERROR: Camera 1 has null image at timestamp 1763729872195076607
ERROR: Camera 1 has null image at timestamp 1763729872490575134
ERROR: Camera 1 has null image at timestamp 1763729872629924760
ERROR: Camera 1 has null image at timestamp 1763729872785263691
ERROR: Camera 1 has null image at timestamp 1763729872929428645
ERROR: Camera 1 has null image at timestamp 1763729873079993736
ERROR: Camera 1 has null image at timestamp 1763729873217896913
ERROR: Camera 1 has null image at timestamp 1763729873388473429
ERROR: Camera 1 has null image at timestamp 1763729873532469363
  Found 144 corners at frame 1263 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729873695633647
  Found 144 corners at frame 1269 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729873835652859
  Found 144 corners at frame 1275 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729874140280500
ERROR: Camera 1 has null image at timestamp 1763729874278779858
ERROR: Camera 1 has null image at timestamp 1763729874422134495
ERROR: Camera 1 has null image at timestamp 1763729874731718407
ERROR: Camera 1 has null image at timestamp 1763729874785238912
ERROR: Camera 1 has null image at timestamp 1763729875027922770
ERROR: Camera 1 has null image at timestamp 1763729875176111871
ERROR: Camera 1 has null image at timestamp 1763729875229134046
ERROR: Camera 1 has null image at timestamp 1763729875325655438
ERROR: Camera 1 has null image at timestamp 1763729875535587455
ERROR: Camera 1 has null image at timestamp 1763729875690561038
ERROR: Camera 1 has null image at timestamp 1763729875845797860
ERROR: Camera 1 has null image at timestamp 1763729875997963056
ERROR: Camera 1 has null image at timestamp 1763729876137742288
ERROR: Camera 1 has null image at timestamp 1763729876233838826
ERROR: Camera 1 has null image at timestamp 1763729876290659274
ERROR: Camera 1 has null image at timestamp 1763729876422018930
ERROR: Camera 1 has null image at timestamp 1763729876584604706
ERROR: Camera 1 has null image at timestamp 1763729876718250253
ERROR: Camera 1 has null image at timestamp 1763729876879993303
ERROR: Camera 1 has null image at timestamp 1763729876976985228
ERROR: Camera 1 has null image at timestamp 1763729877125239215
ERROR: Camera 1 has null image at timestamp 1763729877269968363
ERROR: Camera 1 has null image at timestamp 1763729877418311609
ERROR: Camera 1 has null image at timestamp 1763729877719694486
ERROR: Camera 1 has null image at timestamp 1763729877869546481
ERROR: Camera 1 has null image at timestamp 1763729878018518593
ERROR: Camera 1 has null image at timestamp 1763729878160225564
ERROR: Camera 1 has null image at timestamp 1763729878308818290
ERROR: Camera 1 has null image at timestamp 1763729878462784948
ERROR: Camera 1 has null image at timestamp 1763729878604825978
ERROR: Camera 1 has null image at timestamp 1763729878758307716
ERROR: Camera 1 has null image at timestamp 1763729878893940578
ERROR: Camera 1 has null image at timestamp 1763729879058417146
ERROR: Camera 1 has null image at timestamp 1763729879197852124
ERROR: Camera 1 has null image at timestamp 1763729879352044305
ERROR: Camera 1 has null image at timestamp 1763729879486267526
ERROR: Camera 1 has null image at timestamp 1763729879645080104
ERROR: Camera 1 has null image at timestamp 1763729879933966527
  Found 40 corners at frame 1521 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729880079122907
  Found 44 corners at frame 1527 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729880389230787
ERROR: Camera 1 has null image at timestamp 1763729880538259125
ERROR: Camera 1 has null image at timestamp 1763729880692823082
ERROR: Camera 1 has null image at timestamp 1763729880827645765
ERROR: Camera 1 has null image at timestamp 1763729880988156507
ERROR: Camera 1 has null image at timestamp 1763729881297950095
ERROR: Camera 1 has null image at timestamp 1763729881360397099
  Found 144 corners at frame 1581 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729881609153604
  Found 144 corners at frame 1587 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729881761822231
  Found 144 corners at frame 1593 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729882056792055
ERROR: Camera 1 has null image at timestamp 1763729882198144923
  Found 144 corners at frame 1611 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729882351486488
  Found 140 corners at frame 1617 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729882495647605
  Found 136 corners at frame 1623 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729882654165168
  Found 136 corners at frame 1629 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729882813677644
  Found 132 corners at frame 1635 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 132 corners at frame 1638 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729883102306210
  Found 128 corners at frame 1647 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729883247122036
  Found 132 corners at frame 1653 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729883396972917
ERROR: Camera 1 has null image at timestamp 1763729883540780629
ERROR: Camera 1 has null image at timestamp 1763729883694398213
ERROR: Camera 1 has null image at timestamp 1763729883837236923
ERROR: Camera 1 has null image at timestamp 1763729884139961770
ERROR: Camera 1 has null image at timestamp 1763729884285629053
ERROR: Camera 1 has null image at timestamp 1763729884586693522
  Found 144 corners at frame 1707 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729884718358465
  Found 144 corners at frame 1713 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729884879047378
  Found 144 corners at frame 1719 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729885022919835
ERROR: Camera 1 has null image at timestamp 1763729885181545865
ERROR: Camera 1 has null image at timestamp 1763729885228652755
ERROR: Camera 1 has null image at timestamp 1763729885327188144
  Found 144 corners at frame 1737 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 144 corners at frame 1740 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729885563631116
  Found 144 corners at frame 1746 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729885724470891
  Found 144 corners at frame 1752 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729885867106638
  Found 144 corners at frame 1758 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729886026404977
  Found 144 corners at frame 1764 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729886164628111
ERROR: Camera 1 has null image at timestamp 1763729886233521558
ERROR: Camera 1 has null image at timestamp 1763729886325997785
  Found 144 corners at frame 1776 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 144 corners at frame 1779 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 144 corners at frame 1782 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729886629927985
ERROR: Camera 1 has null image at timestamp 1763729886679125348
ERROR: Camera 1 has null image at timestamp 1763729886929553341
  Found 144 corners at frame 1800 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729887065054971
  Found 144 corners at frame 1806 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729887226958037
ERROR: Camera 1 has null image at timestamp 1763729887273184278
  Found 144 corners at frame 1815 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729887516716738
ERROR: Camera 1 has null image at timestamp 1763729887561770593
ERROR: Camera 1 has null image at timestamp 1763729887665749968
ERROR: Camera 1 has null image at timestamp 1763729887823850500
ERROR: Camera 1 has null image at timestamp 1763729887873156259
  Found 144 corners at frame 1839 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729888074106959
  Found 144 corners at frame 1845 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729888225476019
  Found 144 corners at frame 1851 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729888521370012
ERROR: Camera 1 has null image at timestamp 1763729888681850795
ERROR: Camera 1 has null image at timestamp 1763729888836110938
ERROR: Camera 1 has null image at timestamp 1763729888985843926
ERROR: Camera 1 has null image at timestamp 1763729889036488716
ERROR: Camera 1 has null image at timestamp 1763729889136502683
ERROR: Camera 1 has null image at timestamp 1763729889189804373
ERROR: Camera 1 has null image at timestamp 1763729889339090156
ERROR: Camera 1 has null image at timestamp 1763729889432879013
  Found 0 corners at frame 1899 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729889586484728
  Found 0 corners at frame 1905 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 0 corners at frame 1908 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 0 corners at frame 1911 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729889892260184
  Found 0 corners at frame 1917 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729890058532492
ERROR: Camera 1 has null image at timestamp 1763729890207457745
ERROR: Camera 1 has null image at timestamp 1763729890350251549
ERROR: Camera 1 has null image at timestamp 1763729890413213089
ERROR: Camera 1 has null image at timestamp 1763729890667546008
ERROR: Camera 1 has null image at timestamp 1763729890721982507
ERROR: Camera 1 has null image at timestamp 1763729890816016305
ERROR: Camera 1 has null image at timestamp 1763729890956342834
  Found 0 corners at frame 1959 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729891123197423
ERROR: Camera 1 has null image at timestamp 1763729891173421409
  Found 0 corners at frame 1968 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 0 corners at frame 1971 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729891417285278
ERROR: Camera 1 has null image at timestamp 1763729891462039507
  Found 0 corners at frame 1980 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 1 has null image at timestamp 1763729891616612828
ERROR: Camera 1 has null image at timestamp 1763729891719960474
ERROR: Camera 1 has null image at timestamp 1763729891766183641
ERROR: Camera 1 has null image at timestamp 1763729891861070991
ERROR: Camera 1 has null image at timestamp 1763729891921927741
ERROR: Camera 1 has null image at timestamp 1763729892100263059
ERROR: Camera 1 has null image at timestamp 1763729892166060995
ERROR: Camera 1 has null image at timestamp 1763729892222792257
ERROR: Camera 1 has null image at timestamp 1763729892466655637
  ⚠ Camera 1 not initialized yet, will try pinhole fallback

--- Initializing camera 2 (model: ds) ---
ERROR: Camera 2 has null image at timestamp 1763729842162678522
  Found 144 corners at frame 3 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729842318630659
  Found 144 corners at frame 9 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 144 corners at frame 12 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 144 corners at frame 15 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729842619927874
ERROR: Camera 2 has null image at timestamp 1763729842772134190
ERROR: Camera 2 has null image at timestamp 1763729842914143704
ERROR: Camera 2 has null image at timestamp 1763729843073335022
ERROR: Camera 2 has null image at timestamp 1763729843226434002
ERROR: Camera 2 has null image at timestamp 1763729843272673017
ERROR: Camera 2 has null image at timestamp 1763729843362045826
ERROR: Camera 2 has null image at timestamp 1763729843516390387
ERROR: Camera 2 has null image at timestamp 1763729843667779960
ERROR: Camera 2 has null image at timestamp 1763729843817548406
ERROR: Camera 2 has null image at timestamp 1763729843866690237
ERROR: Camera 2 has null image at timestamp 1763729844116095902
ERROR: Camera 2 has null image at timestamp 1763729844247508974
ERROR: Camera 2 has null image at timestamp 1763729844399193843
ERROR: Camera 2 has null image at timestamp 1763729844552953958
ERROR: Camera 2 has null image at timestamp 1763729844855018052
ERROR: Camera 2 has null image at timestamp 1763729844947824414
ERROR: Camera 2 has null image at timestamp 1763729845152844699
ERROR: Camera 2 has null image at timestamp 1763729845299029431
ERROR: Camera 2 has null image at timestamp 1763729845441688976
ERROR: Camera 2 has null image at timestamp 1763729845592686730
ERROR: Camera 2 has null image at timestamp 1763729845741071936
ERROR: Camera 2 has null image at timestamp 1763729845894684194
ERROR: Camera 2 has null image at timestamp 1763729846035514201
ERROR: Camera 2 has null image at timestamp 1763729846431052149
ERROR: Camera 2 has null image at timestamp 1763729846492721653
ERROR: Camera 2 has null image at timestamp 1763729846535205279
ERROR: Camera 2 has null image at timestamp 1763729846640160766
ERROR: Camera 2 has null image at timestamp 1763729846699488085
ERROR: Camera 2 has null image at timestamp 1763729846794080544
ERROR: Camera 2 has null image at timestamp 1763729846956862772
ERROR: Camera 2 has null image at timestamp 1763729847110572043
ERROR: Camera 2 has null image at timestamp 1763729847252337458
ERROR: Camera 2 has null image at timestamp 1763729847412680420
ERROR: Camera 2 has null image at timestamp 1763729847566634630
ERROR: Camera 2 has null image at timestamp 1763729847717849737
ERROR: Camera 2 has null image at timestamp 1763729847812682665
ERROR: Camera 2 has null image at timestamp 1763729847868324438
ERROR: Camera 2 has null image at timestamp 1763729848023946360
ERROR: Camera 2 has null image at timestamp 1763729848169796685
ERROR: Camera 2 has null image at timestamp 1763729848319739531
ERROR: Camera 2 has null image at timestamp 1763729848476915391
ERROR: Camera 2 has null image at timestamp 1763729848613104540
ERROR: Camera 2 has null image at timestamp 1763729848759084773
ERROR: Camera 2 has null image at timestamp 1763729848924685657
ERROR: Camera 2 has null image at timestamp 1763729849019239424
ERROR: Camera 2 has null image at timestamp 1763729849072691199
ERROR: Camera 2 has null image at timestamp 1763729849212549411
ERROR: Camera 2 has null image at timestamp 1763729849273403499
ERROR: Camera 2 has null image at timestamp 1763729849361184238
ERROR: Camera 2 has null image at timestamp 1763729849679695740
ERROR: Camera 2 has null image at timestamp 1763729849816788589
ERROR: Camera 2 has null image at timestamp 1763729849884543307
ERROR: Camera 2 has null image at timestamp 1763729849976025098
ERROR: Camera 2 has null image at timestamp 1763729850031915582
ERROR: Camera 2 has null image at timestamp 1763729850121685031
ERROR: Camera 2 has null image at timestamp 1763729850277581368
ERROR: Camera 2 has null image at timestamp 1763729850427089443
ERROR: Camera 2 has null image at timestamp 1763729850563027787
ERROR: Camera 2 has null image at timestamp 1763729850629854222
ERROR: Camera 2 has null image at timestamp 1763729850719378452
ERROR: Camera 2 has null image at timestamp 1763729850873031362
ERROR: Camera 2 has null image at timestamp 1763729851024313240
ERROR: Camera 2 has null image at timestamp 1763729851163170952
ERROR: Camera 2 has null image at timestamp 1763729851316798299
ERROR: Camera 2 has null image at timestamp 1763729851466307702
ERROR: Camera 2 has null image at timestamp 1763729851594498429
ERROR: Camera 2 has null image at timestamp 1763729851759430149
ERROR: Camera 2 has null image at timestamp 1763729851900770850
ERROR: Camera 2 has null image at timestamp 1763729852056430281
ERROR: Camera 2 has null image at timestamp 1763729852191854999
ERROR: Camera 2 has null image at timestamp 1763729852341339817
ERROR: Camera 2 has null image at timestamp 1763729852403805394
ERROR: Camera 2 has null image at timestamp 1763729852644596907
ERROR: Camera 2 has null image at timestamp 1763729852729259715
ERROR: Camera 2 has null image at timestamp 1763729852785350020
ERROR: Camera 2 has null image at timestamp 1763729852927423096
ERROR: Camera 2 has null image at timestamp 1763729853084715317
ERROR: Camera 2 has null image at timestamp 1763729853218538955
ERROR: Camera 2 has null image at timestamp 1763729853379750697
ERROR: Camera 2 has null image at timestamp 1763729853536855879
ERROR: Camera 2 has null image at timestamp 1763729853687642711
ERROR: Camera 2 has null image at timestamp 1763729853989623840
ERROR: Camera 2 has null image at timestamp 1763729854134596520
ERROR: Camera 2 has null image at timestamp 1763729854294778706
ERROR: Camera 2 has null image at timestamp 1763729854382634947
ERROR: Camera 2 has null image at timestamp 1763729854443056009
ERROR: Camera 2 has null image at timestamp 1763729854579200112
ERROR: Camera 2 has null image at timestamp 1763729854645834550
ERROR: Camera 2 has null image at timestamp 1763729854739304354
  Found 84 corners at frame 507 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729854887009154
ERROR: Camera 2 has null image at timestamp 1763729855037929456
ERROR: Camera 2 has null image at timestamp 1763729855192328920
ERROR: Camera 2 has null image at timestamp 1763729855235394130
ERROR: Camera 2 has null image at timestamp 1763729855486224045
ERROR: Camera 2 has null image at timestamp 1763729855619828725
ERROR: Camera 2 has null image at timestamp 1763729855788182895
ERROR: Camera 2 has null image at timestamp 1763729855875521251
ERROR: Camera 2 has null image at timestamp 1763729855933131410
ERROR: Camera 2 has null image at timestamp 1763729856082871088
ERROR: Camera 2 has null image at timestamp 1763729856232103146
ERROR: Camera 2 has null image at timestamp 1763729856530269731
ERROR: Camera 2 has null image at timestamp 1763729856683776881
ERROR: Camera 2 has null image at timestamp 1763729856830917419
ERROR: Camera 2 has null image at timestamp 1763729856976811540
ERROR: Camera 2 has null image at timestamp 1763729857122555498
ERROR: Camera 2 has null image at timestamp 1763729857272655915
ERROR: Camera 2 has null image at timestamp 1763729857428661141
ERROR: Camera 2 has null image at timestamp 1763729857566195470
ERROR: Camera 2 has null image at timestamp 1763729857727631969
ERROR: Camera 2 has null image at timestamp 1763729857864238993
ERROR: Camera 2 has null image at timestamp 1763729858117736494
ERROR: Camera 2 has null image at timestamp 1763729858177969050
ERROR: Camera 2 has null image at timestamp 1763729858223991205
ERROR: Camera 2 has null image at timestamp 1763729858472603815
ERROR: Camera 2 has null image at timestamp 1763729858629124204
ERROR: Camera 2 has null image at timestamp 1763729858769257602
ERROR: Camera 2 has null image at timestamp 1763729858830193897
ERROR: Camera 2 has null image at timestamp 1763729858918644433
ERROR: Camera 2 has null image at timestamp 1763729859069647014
ERROR: Camera 2 has null image at timestamp 1763729859203206019
ERROR: Camera 2 has null image at timestamp 1763729859368401540
ERROR: Camera 2 has null image at timestamp 1763729859497241950
ERROR: Camera 2 has null image at timestamp 1763729859656423428
ERROR: Camera 2 has null image at timestamp 1763729859798353958
ERROR: Camera 2 has null image at timestamp 1763729859942021903
ERROR: Camera 2 has null image at timestamp 1763729860091072860
ERROR: Camera 2 has null image at timestamp 1763729860137761318
ERROR: Camera 2 has null image at timestamp 1763729860325722733
ERROR: Camera 2 has null image at timestamp 1763729860386393486
ERROR: Camera 2 has null image at timestamp 1763729860490522385
ERROR: Camera 2 has null image at timestamp 1763729860641762572
ERROR: Camera 2 has null image at timestamp 1763729860784190594
ERROR: Camera 2 has null image at timestamp 1763729860936115522
ERROR: Camera 2 has null image at timestamp 1763729861086005993
ERROR: Camera 2 has null image at timestamp 1763729861245507912
ERROR: Camera 2 has null image at timestamp 1763729861372928484
ERROR: Camera 2 has null image at timestamp 1763729861530189909
ERROR: Camera 2 has null image at timestamp 1763729861616900167
ERROR: Camera 2 has null image at timestamp 1763729861675600089
ERROR: Camera 2 has null image at timestamp 1763729861830046211
ERROR: Camera 2 has null image at timestamp 1763729861963432425
ERROR: Camera 2 has null image at timestamp 1763729862054035175
ERROR: Camera 2 has null image at timestamp 1763729862204032716
ERROR: Camera 2 has null image at timestamp 1763729862358220768
ERROR: Camera 2 has null image at timestamp 1763729862523443531
ERROR: Camera 2 has null image at timestamp 1763729862664583858
ERROR: Camera 2 has null image at timestamp 1763729862756012921
ERROR: Camera 2 has null image at timestamp 1763729862802451480
ERROR: Camera 2 has null image at timestamp 1763729862950192183
ERROR: Camera 2 has null image at timestamp 1763729863053292436
ERROR: Camera 2 has null image at timestamp 1763729863198973398
ERROR: Camera 2 has null image at timestamp 1763729863354805067
ERROR: Camera 2 has null image at timestamp 1763729863501430448
ERROR: Camera 2 has null image at timestamp 1763729863809911039
ERROR: Camera 2 has null image at timestamp 1763729863953117253
ERROR: Camera 2 has null image at timestamp 1763729864106218130
ERROR: Camera 2 has null image at timestamp 1763729864253924471
ERROR: Camera 2 has null image at timestamp 1763729864315465468
ERROR: Camera 2 has null image at timestamp 1763729864405596779
ERROR: Camera 2 has null image at timestamp 1763729864550438019
ERROR: Camera 2 has null image at timestamp 1763729864687585422
ERROR: Camera 2 has null image at timestamp 1763729864850285175
ERROR: Camera 2 has null image at timestamp 1763729864896517838
ERROR: Camera 2 has null image at timestamp 1763729865138525049
ERROR: Camera 2 has null image at timestamp 1763729865446668649
ERROR: Camera 2 has null image at timestamp 1763729865588450626
ERROR: Camera 2 has null image at timestamp 1763729865725200342
ERROR: Camera 2 has null image at timestamp 1763729865877864421
ERROR: Camera 2 has null image at timestamp 1763729866033567301
ERROR: Camera 2 has null image at timestamp 1763729866188105967
ERROR: Camera 2 has null image at timestamp 1763729866327039746
ERROR: Camera 2 has null image at timestamp 1763729866385401977
ERROR: Camera 2 has null image at timestamp 1763729866486038867
ERROR: Camera 2 has null image at timestamp 1763729866577787464
ERROR: Camera 2 has null image at timestamp 1763729866725189849
ERROR: Camera 2 has null image at timestamp 1763729866815792950
ERROR: Camera 2 has null image at timestamp 1763729867019284525
ERROR: Camera 2 has null image at timestamp 1763729867156902461
ERROR: Camera 2 has null image at timestamp 1763729867300792511
  Found 144 corners at frame 1011 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729867461902359
  Found 144 corners at frame 1017 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729867608708006
  Found 144 corners at frame 1023 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729867756431111
  Found 144 corners at frame 1029 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729867904760593
ERROR: Camera 2 has null image at timestamp 1763729868055728047
ERROR: Camera 2 has null image at timestamp 1763729868287388295
ERROR: Camera 2 has null image at timestamp 1763729868345708412
ERROR: Camera 2 has null image at timestamp 1763729868392598577
ERROR: Camera 2 has null image at timestamp 1763729868492165891
ERROR: Camera 2 has null image at timestamp 1763729868537284698
ERROR: Camera 2 has null image at timestamp 1763729868629927355
ERROR: Camera 2 has null image at timestamp 1763729868886898787
ERROR: Camera 2 has null image at timestamp 1763729868941463933
ERROR: Camera 2 has null image at timestamp 1763729868989751727
ERROR: Camera 2 has null image at timestamp 1763729869232339547
ERROR: Camera 2 has null image at timestamp 1763729869382552277
ERROR: Camera 2 has null image at timestamp 1763729869679609279
ERROR: Camera 2 has null image at timestamp 1763729869819633378
ERROR: Camera 2 has null image at timestamp 1763729869963400086
ERROR: Camera 2 has null image at timestamp 1763729870123061740
ERROR: Camera 2 has null image at timestamp 1763729870170005895
ERROR: Camera 2 has null image at timestamp 1763729870255395703
ERROR: Camera 2 has null image at timestamp 1763729870411686245
  Found 144 corners at frame 1137 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729870552970522
ERROR: Camera 2 has null image at timestamp 1763729870695733237
ERROR: Camera 2 has null image at timestamp 1763729870860289910
ERROR: Camera 2 has null image at timestamp 1763729871005868556
ERROR: Camera 2 has null image at timestamp 1763729871154940960
ERROR: Camera 2 has null image at timestamp 1763729871291065724
ERROR: Camera 2 has null image at timestamp 1763729871453926539
ERROR: Camera 2 has null image at timestamp 1763729871545395485
ERROR: Camera 2 has null image at timestamp 1763729871595063169
ERROR: Camera 2 has null image at timestamp 1763729871749423808
ERROR: Camera 2 has null image at timestamp 1763729871842874410
ERROR: Camera 2 has null image at timestamp 1763729871990349505
ERROR: Camera 2 has null image at timestamp 1763729872195076607
ERROR: Camera 2 has null image at timestamp 1763729872490575134
ERROR: Camera 2 has null image at timestamp 1763729872629924760
ERROR: Camera 2 has null image at timestamp 1763729872785263691
ERROR: Camera 2 has null image at timestamp 1763729872929428645
ERROR: Camera 2 has null image at timestamp 1763729873022683758
ERROR: Camera 2 has null image at timestamp 1763729873079993736
ERROR: Camera 2 has null image at timestamp 1763729873217896913
ERROR: Camera 2 has null image at timestamp 1763729873388473429
ERROR: Camera 2 has null image at timestamp 1763729873532469363
  Found 144 corners at frame 1263 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729873695633647
  Found 144 corners at frame 1269 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729873835652859
  Found 144 corners at frame 1275 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 144 corners at frame 1278 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 144 corners at frame 1281 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729874140280500
ERROR: Camera 2 has null image at timestamp 1763729874278779858
ERROR: Camera 2 has null image at timestamp 1763729874422134495
ERROR: Camera 2 has null image at timestamp 1763729874731718407
ERROR: Camera 2 has null image at timestamp 1763729874785238912
ERROR: Camera 2 has null image at timestamp 1763729875027922770
ERROR: Camera 2 has null image at timestamp 1763729875176111871
ERROR: Camera 2 has null image at timestamp 1763729875229134046
ERROR: Camera 2 has null image at timestamp 1763729875325655438
ERROR: Camera 2 has null image at timestamp 1763729875535587455
ERROR: Camera 2 has null image at timestamp 1763729875690561038
ERROR: Camera 2 has null image at timestamp 1763729875845797860
ERROR: Camera 2 has null image at timestamp 1763729875997963056
ERROR: Camera 2 has null image at timestamp 1763729876137742288
ERROR: Camera 2 has null image at timestamp 1763729876290659274
ERROR: Camera 2 has null image at timestamp 1763729876422018930
ERROR: Camera 2 has null image at timestamp 1763729876584604706
ERROR: Camera 2 has null image at timestamp 1763729876718250253
ERROR: Camera 2 has null image at timestamp 1763729876819300687
ERROR: Camera 2 has null image at timestamp 1763729876879993303
ERROR: Camera 2 has null image at timestamp 1763729876976985228
ERROR: Camera 2 has null image at timestamp 1763729877125239215
ERROR: Camera 2 has null image at timestamp 1763729877269968363
ERROR: Camera 2 has null image at timestamp 1763729877418311609
ERROR: Camera 2 has null image at timestamp 1763729877719694486
ERROR: Camera 2 has null image at timestamp 1763729877869546481
ERROR: Camera 2 has null image at timestamp 1763729878018518593
ERROR: Camera 2 has null image at timestamp 1763729878160225564
ERROR: Camera 2 has null image at timestamp 1763729878308818290
ERROR: Camera 2 has null image at timestamp 1763729878462784948
ERROR: Camera 2 has null image at timestamp 1763729878604825978
ERROR: Camera 2 has null image at timestamp 1763729878758307716
ERROR: Camera 2 has null image at timestamp 1763729878893940578
ERROR: Camera 2 has null image at timestamp 1763729879058417146
ERROR: Camera 2 has null image at timestamp 1763729879197852124
ERROR: Camera 2 has null image at timestamp 1763729879352044305
ERROR: Camera 2 has null image at timestamp 1763729879486267526
ERROR: Camera 2 has null image at timestamp 1763729879645080104
ERROR: Camera 2 has null image at timestamp 1763729879695832800
ERROR: Camera 2 has null image at timestamp 1763729879933966527
  Found 16 corners at frame 1521 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729880079122907
  Found 32 corners at frame 1527 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 32 corners at frame 1530 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 40 corners at frame 1533 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729880389230787
ERROR: Camera 2 has null image at timestamp 1763729880538259125
ERROR: Camera 2 has null image at timestamp 1763729880692823082
ERROR: Camera 2 has null image at timestamp 1763729880827645765
ERROR: Camera 2 has null image at timestamp 1763729880988156507
ERROR: Camera 2 has null image at timestamp 1763729881297950095
ERROR: Camera 2 has null image at timestamp 1763729881360397099
  Found 140 corners at frame 1581 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729881609153604
  Found 136 corners at frame 1587 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729881761822231
  Found 132 corners at frame 1593 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 136 corners at frame 1596 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 139 corners at frame 1599 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729882056792055
  Found 40 corners at frame 1605 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729882198144923
  Found 96 corners at frame 1611 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729882351486488
  Found 144 corners at frame 1617 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729882495647605
  Found 144 corners at frame 1623 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729882654165168
  Found 140 corners at frame 1629 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729882813677644
  Found 136 corners at frame 1635 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 136 corners at frame 1638 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 136 corners at frame 1641 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729883102306210
  Found 132 corners at frame 1647 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729883247122036
  Found 132 corners at frame 1653 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729883396972917
  Found 136 corners at frame 1659 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729883540780629
ERROR: Camera 2 has null image at timestamp 1763729883694398213
ERROR: Camera 2 has null image at timestamp 1763729883837236923
ERROR: Camera 2 has null image at timestamp 1763729884139961770
ERROR: Camera 2 has null image at timestamp 1763729884285629053
ERROR: Camera 2 has null image at timestamp 1763729884527636559
ERROR: Camera 2 has null image at timestamp 1763729884586693522
  Found 144 corners at frame 1707 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729884718358465
  Found 144 corners at frame 1713 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729884879047378
  Found 144 corners at frame 1719 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729885022919835
  Found 144 corners at frame 1725 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729885181545865
ERROR: Camera 2 has null image at timestamp 1763729885228652755
ERROR: Camera 2 has null image at timestamp 1763729885327188144
  Found 144 corners at frame 1737 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 144 corners at frame 1740 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729885563631116
  Found 144 corners at frame 1746 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729885724470891
  Found 144 corners at frame 1752 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729885867106638
  Found 144 corners at frame 1758 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729886026404977
  Found 143 corners at frame 1764 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729886164628111
ERROR: Camera 2 has null image at timestamp 1763729886233521558
ERROR: Camera 2 has null image at timestamp 1763729886325997785
  Found 144 corners at frame 1776 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 144 corners at frame 1779 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 144 corners at frame 1782 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729886629927985
ERROR: Camera 2 has null image at timestamp 1763729886679125348
  Found 144 corners at frame 1791 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 144 corners at frame 1794 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729886929553341
  Found 144 corners at frame 1800 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729887065054971
  Found 144 corners at frame 1806 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729887226958037
ERROR: Camera 2 has null image at timestamp 1763729887273184278
  Found 144 corners at frame 1815 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 144 corners at frame 1818 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729887516716738
ERROR: Camera 2 has null image at timestamp 1763729887561770593
ERROR: Camera 2 has null image at timestamp 1763729887665749968
ERROR: Camera 2 has null image at timestamp 1763729887823850500
ERROR: Camera 2 has null image at timestamp 1763729887873156259
  Found 144 corners at frame 1839 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729888074106959
ERROR: Camera 2 has null image at timestamp 1763729888162042970
ERROR: Camera 2 has null image at timestamp 1763729888225476019
  Found 144 corners at frame 1851 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 144 corners at frame 1854 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 144 corners at frame 1857 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729888521370012
  Found 132 corners at frame 1863 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729888681850795
  Found 120 corners at frame 1869 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729888836110938
ERROR: Camera 2 has null image at timestamp 1763729888985843926
ERROR: Camera 2 has null image at timestamp 1763729889036488716
ERROR: Camera 2 has null image at timestamp 1763729889136502683
ERROR: Camera 2 has null image at timestamp 1763729889189804373
ERROR: Camera 2 has null image at timestamp 1763729889339090156
ERROR: Camera 2 has null image at timestamp 1763729889432879013
  Found 100 corners at frame 1899 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729889586484728
ERROR: Camera 2 has null image at timestamp 1763729889672403323
  Found 0 corners at frame 1908 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 0 corners at frame 1911 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729889892260184
ERROR: Camera 2 has null image at timestamp 1763729889986514476
ERROR: Camera 2 has null image at timestamp 1763729890058532492
  Found 0 corners at frame 1923 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729890207457745
  Found 0 corners at frame 1929 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729890350251549
ERROR: Camera 2 has null image at timestamp 1763729890413213089
ERROR: Camera 2 has null image at timestamp 1763729890667546008
ERROR: Camera 2 has null image at timestamp 1763729890721982507
ERROR: Camera 2 has null image at timestamp 1763729890816016305
ERROR: Camera 2 has null image at timestamp 1763729890956342834
  Found 0 corners at frame 1959 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729891123197423
ERROR: Camera 2 has null image at timestamp 1763729891173421409
  Found 0 corners at frame 1968 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729891353009642
ERROR: Camera 2 has null image at timestamp 1763729891417285278
ERROR: Camera 2 has null image at timestamp 1763729891462039507
  Found 0 corners at frame 1980 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729891616612828
ERROR: Camera 2 has null image at timestamp 1763729891719960474
ERROR: Camera 2 has null image at timestamp 1763729891766183641
ERROR: Camera 2 has null image at timestamp 1763729891861070991
ERROR: Camera 2 has null image at timestamp 1763729891921927741
  Found 0 corners at frame 1998 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 0 corners at frame 2001 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729892166060995
ERROR: Camera 2 has null image at timestamp 1763729892222792257
ERROR: Camera 2 has null image at timestamp 1763729892466655637
ERROR: Camera 2 has null image at timestamp 1763729892552717970
  ⚠ Camera 2 not initialized yet, will try pinhole fallback

...
=== Pinhole fallback initialization ===
Trying pinhole initialization for camera 0
ERROR: Null image for camera 0
...
ERROR: Null image for camera 0
  Pinhole corners collected: 312 (image: 1920x1080)
Initialized camera 0 with pinhole model. You should set pinhole model for this camera!
Trying pinhole initialization for camera 1
ERROR: Null image for camera 1
...
ERROR: Null image for camera 1
  Pinhole corners collected: 319 (image: 1280x800)
Initialized camera 1 with pinhole model. You should set pinhole model for this camera!
Trying pinhole initialization for camera 2
ERROR: Null image for camera 2
...
ERROR: Null image for camera 2
  Pinhole corners collected: 303 (image: 1280x800)
Initialized camera 2 with pinhole model. You should set pinhole model for this camera!
Done camera intrinsics initialization:
Cam 0: 864.344 861.298   959.5   539.5       0     0.5
Cam 1: 935.542 933.364   639.5   399.5       0     0.5
Cam 2: 957.543 952.788   639.5   399.5       0     0.5

=== Started camera intrinsics initialization ===
Number of cameras: 3
Number of camera types specified: 3
Camera models: ds ds ds 
Total corner detections across all cameras: 2963
  Camera 0: 1014 detections
  Camera 1: 1001 detections
  Camera 2: 948 detections
Calling resetCalib...
resetCalib completed successfully

--- Initializing camera 0 (model: ds) ---
  Found 144 corners at frame 0 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
ERROR: Camera 0 has null image at timestamp 1763729842259453643
  Found 144 corners at frame 6 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
ERROR: Camera 0 has null image at timestamp 1763729842414387749
ERROR: Camera 0 has null image at timestamp 1763729842477201141
ERROR: Camera 0 has null image at timestamp 1763729842565827258
  Found 144 corners at frame 18 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
...
ERROR: Camera 0 has null image at timestamp 1763729845004684178
ERROR: Camera 0 has null image at timestamp 1763729845088208263
  Found 144 corners at frame 120 (image size: 1920x1080)
  × Initialization failed for this frame, trying next...
...
ERROR: Camera 2 has null image at timestamp 1763729892222792257
  Found 0 corners at frame 2010 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
  Found 0 corners at frame 2013 (image size: 1280x800)
  × Initialization failed for this frame, trying next...
ERROR: Camera 2 has null image at timestamp 1763729892466655637
ERROR: Camera 2 has null image at timestamp 1763729892552717970
  ⚠ Camera 2 not initialized yet, will try pinhole fallback

=== Pinhole fallback initialization ===
Trying pinhole initialization for camera 0
ERROR: Null image for camera 0
...
ERROR: Null image for camera 0
  Pinhole corners collected: 312 (image: 1920x1080)
Initialized camera 0 with pinhole model. You should set pinhole model for this camera!
Trying pinhole initialization for camera 1
ERROR: Null image for camera 1
...
ERROR: Null image for camera 1
  Pinhole corners collected: 319 (image: 1280x800)
Initialized camera 1 with pinhole model. You should set pinhole model for this camera!
Trying pinhole initialization for camera 2
ERROR: Null image for camera 2
ERROR: Null image for camera 2
...
ERROR: Null image for camera 2
  Pinhole corners collected: 303 (image: 1280x800)
Initialized camera 2 with pinhole model. You should set pinhole model for this camera!
Done camera intrinsics initialization:
Cam 0: 864.344 861.298   959.5   539.5       0     0.5
Cam 1: 935.542 933.364   639.5   399.5       0     0.5
Cam 2: 957.543 952.788   639.5   399.5       0     0.5

=== Started initial camera pose computation ===
Checking camera initialization status:
  Camera 0: ✓ VALID (fx=864.344, fy=861.298)
  Camera 1: ✓ VALID (fx=935.542, fy=933.364)
  Camera 2: ✓ VALID (fx=957.543, fy=952.788)
All cameras initialized successfully, computing poses...
Segmentation fault (core dumped)