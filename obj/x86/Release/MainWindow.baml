   M S B A M L   `   `   `  ����   DTWGestureRecognition&     DTWGestureRecognition.MainWindow   \ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     -     � Title$  	KinectDTW��$	��674��$	��620��.���  � RowDefinitions �� $��80 ��� $	��140 ��� $	��150 ��� $	��250 ��� $��16 � � ColumnDefinitions �� $	��200 ��� $	��400 �.+��� $��0��$��0��$��2��.+��� $3��-/DTWGestureRecognition;component/HeaderBG.png���� $��40��$?��9Kinect SDK Dynamic Time Warping (DTW) Gesture Recognition��$��14(�$
��Bold"�$��Center=�$��Center=�$	��600��$��SemiExpanded&�$��Normal$�$��Center=�$	��Top=�$��Segoe UI)��� $��0��$��1��.+��� -   $��dtwRead��$
��Read��$	��200��{�.  �� -   $��
dtwCapture��$��Capture��$
��200 ���� -   $��dtwStore��$��Store��{�.  $	��200���� -   $��dtwLoadFile��$��Load gesture file��{�. $	��200���� -   $��dtwSaveToFile��$��Save to file��{�. $	��200���� -   $��dtwShowGestureTest��$��Show Gesture Text��{�. $	��200���� $��1��$��1��$
��Left=�.+��� -   $��gestureList��$	��400��.@���  �� 
IsSelected . .��@Right hand swipe left�� .��@Right hand swipe right�� .��@Left hand swipe left�� .��@Left hand swipe right�� .��@Two hands zoom in�� .��@Two hands zoom out�� .��@Right hand wave�� .��@Left hand wave�� .��@Right hand pull down�� .��@Left hand pull down�� .��@Right hand push up�� .��@Left hand push up�� .��@Both hands pull down�� .��@Both hands push up�� .��@Get on down at the disco!�� -	   $��results��$��14(�$��0,0,0,0q�|�.  $��0q�$�� �� �� TextAlignment$
 Left=�$
��Left=�$	��Top=�$	��400��$��20���� -
   $��status��$��14(�$��0,0,0,0q�|�.  $��0q�$�� ��$
 Left=�$
��Left=�$	��Top=�$	��400��$��20���� -   $��currentBufferFrame��$��14(�$��0,0,0,0q�|�.  $��0q�$�� ��$
 Left=�$
��Left=�$	��Top=�$	��400��$��20���� -   $��	frameRate��$��14(�$��0,0,0,0q�|�.  $��0q�$�� ��$
 Left=�$
��Left=�$	��Top=�$	��400��$��20����  �� Orientation$ 
Horizontal=�$��0��$��2��$��2��$	��Top=�.+��� -   $��
videoImage��$	��200��$	��150��$��0,0,0,0q��� -   $��skeletonCanvas��$	��200��$	��150��
���   �}�. $��0,0,0,0q��� -   $��
depthImage��$	��200��$	��150��$��0,0,0,0q��� -   $��dtwTextOutput��$��0��$��3��$��11(�$��0,0,0,0q�|�. $��0q�$�� ��$
 Left=�$��Center=�$	��Top=�$	��200��$	��230��" �� VerticalScrollBarVisibility$
 Auto=� �� 
IsReadOnly . �� -   $��helpText��$��1��$��3��$��11(�$��0,0,0,0q�|�.  �� TextAlignment$
 Left=�$��Center=�$	��Top=�$	��400��$	��230��	 �� Padding$	 10q�$��WrapWithOverflow=�FD1. To start, step into frame and allow Kinect to track your skeleton�� :82. Select a gesture you wish to record from the ComboBox�� EC3. Perform a gesture over 32 frames - watch the frame counter above�� FD4. Your gesture will be stored automatically and return to read mode�� PN5. Test your gesture. If you're unhappy, record the gesture again to overwrite�� -+6. Record some other gestures and test them�� =;7. Optionally save your gestures to file for future loading�� HF8. Improve this script, add your own gestures, share and enjoy Kinect!�� $��0��$��4��$��2��$3��-By Rhemyst and Rymix - kinectdtw.codeplex.com��
�������$	��Top=�$��Center=�$��16��$��10(�$��0q����� 
 �� EndPoint$
 0.5,1&� �� 
StartPoint$ 0.5,0&�.���  � Color$ 	#FF4D35AB�� � Offset$ 1Q�� $ 	#FF724798��$ 0.498Q�� $ 	#FF2B9E32��$ 0Q�