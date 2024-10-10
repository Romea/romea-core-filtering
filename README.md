# ROMEA Core Filtering Library

This library is a **header-only library** that provides tools for setting up asynchronous data fusion using **Kalman filters** or **particle filters**. It is an integral part of the **ROMEA robotics ecosystem**, enabling enhanced sensor data integration and estimation capabilities.

## **Usage**

1. create a ROS workspace
2. cd worskpace
3. mkdir src
4. wget https://raw.githubusercontent.com/Romea/romea-core-filtering/refs/heads/main/romea_filtering_public.repos
5. vcs import src < romea_filter_public.repos
6. build packages
   - catkin build for ROS1
   - colcon build for ROS2
7. create your application using this library

## **Contributing**

If you'd like to contribute to Project Title, here are some guidelines:

1. Fork the repository.
2. Create a new branch for your changes.
3. Make your changes.
4. Write tests to cover your changes.
5. Run the tests to ensure they pass.
6. Commit your changes.
7. Push your changes to your forked repository.
8. Submit a pull request.

## **License**

Project Title is released under the Apache License 2.0. See the LICENSE file for details.

## **Authors**

The Romea Core Filtering library was written by **Jean Laneurit**, based on his thesis work under the supervision of **Roland Chapuis**. Several individuals contributed scientifically to this project:

**Jean Laneurit**  
**Roland Chapuis**  
**Romuald Aufrere**  
**Christophe Debain**  

## **Contact**

If you have any questions or comments about Romea Core Filtering library, please contact **[Jean Laneurit](mailto:jean.laneurit@inrae.fr)** 