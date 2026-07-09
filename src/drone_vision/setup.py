import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'drone_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryder',
    maintainer_email='ryder@todo.todo',
    description='Vision pipeline: YOLO + OpenCV for cylinder detection, hazard classification, precision landing',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
        'inference': ['onnxruntime-gpu>=1.16.0', 'opencv-python>=4.8.0'],
        'training': ['ultralytics>=8.1.0', 'torch>=2.0.0'],
    },
    entry_points={
        'console_scripts': [
            # 相机驱动
            'camera_node = drone_vision.camera:main',
            # YOLO + OpenCV 视觉管线
            'cylinder_detector = drone_vision.cylinder_detector:main',
            'hazard_classifier = drone_vision.hazard_classifier:main',
            'landing_detector = drone_vision.landing_detector:main',
            # 统一视觉管线 (单节点运行所有检测器)
            'vision_pipeline = drone_vision.vision_pipeline:main',
            # 视觉伺服节点 (可选, 独立运行时订阅 LandingTarget 发布 velocity_cmd)
            'visual_servo = drone_vision.visual_servoing:servo_main',
        ],
    },
)
