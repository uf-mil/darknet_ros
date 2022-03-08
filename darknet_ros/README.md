This is the MIL-modified darknet_ros package which utilizes yolov4-tiny to
detect our buoys.

There are five important files that you need to be aware of in order
to make any modifications.

1. config/ros.yaml

    This file defines:
    - the camera topic we want to listen to
    - darknet topics (including the bounding_boxes topic)
    - enable_opencv (if we want to show the images and bounding boxes in an opencv window)
    - enable_console_output (if we want to constantly output our detections to terminal)

2. config/yolov4-tiny.yaml
    This file defines:
    - The name of the cfg file
    - The name of the weights file
    - The threshold required for something to be considered "detected" (my guess)
    - The list of classes were are inferencing from

3. yolo_network_config/cfg/yolov4-tiny.cfg
    This file is defined by the cfg file you used during the training of the model.
    This will be referenced in the yaml file.

4. yolo_network_config/weights/yolov4-tiny.weights
    This is the file that was generated from training. I chose the best-weights file.
    This will be referenced in the yaml file.

5. launch/darknet_ros.launch
    There are a few parameters here to take into account:
    - The image topic we choose to listen to is also defined here
    - The path to the cfg and weights file
    - The path to the ros.yaml
    - The path to the yolov4-tiny.yaml

Important Note:
    I launch darknet_ros.launch from vrx_classifier.launch.
    In addition to this, I have defined in navigator_launch/config/darknet_params:
        - ros.yaml
        - yolov4-tiny.cfg
        - yolov4-tiny.yaml
    These are referenced in vrx_classifier.launch, and override the args in darknet_ros.

Important Note 2:
    The file yolov4-tiny.weights is in the gitignore due to its size.
    Therefore, I set it up such that this file is downloaded during compilation.
    
    If you wish to replace this file, you will have to do a few things:
        - Test the weights file by creating an argument for its path in vrx_classifier.launch.
            - This will override the current weights file.
        - Once you have verified that it works how you want it to work continue to next step.
        - Upload the new weights file to a git release in the uf-mil/darknet_ros repository.
        - Include the download link of that weights file in the CMakeLists.txt of darknet_ros.
        - Push your changes
        - Note: if you currently have a weights file in the yolo_network_config/cfg directory,
            it will not overwrite it. You will have to delete it and recompile.
    
    If you wish to replace the yolov4-tiny.cfg:
        - Test that it works by changing the cfg file in navigator_launch/config
        - If it does what you want, cp it to the cfg in the darknet_ros package so that it can be
            the default cfg.
