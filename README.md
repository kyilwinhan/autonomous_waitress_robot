<!DOCTYPE html>
<html lang="en">

<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Thesis Presentation | Autonomous Waitress Robot</title>

    <style>
        body {
            font-family: Arial, Helvetica, sans-serif;
            margin: 0;
            padding: 0;
            background: #f4f7f9;
            color: #222;
            line-height: 1.6;
        }

        header {
            background: #0f766e;
            color: #fff;
            padding: 25px;
            text-align: center;
        }

        header h1 {
            margin: 0 0 10px;
            font-size: 28px;
        }

        header p {
            margin: 0;
            font-size: 15px;
            opacity: 0.9;
        }

        .container {
            max-width: 900px;
            margin: auto;
            padding: 20px;
        }

        h2 {
            border-left: 4px solid #0f766e;
            padding-left: 10px;
            margin-top: 40px;
        }

        .card {
            background: #fff;
            padding: 20px;
            margin: 20px 0;
            border-radius: 8px;
            box-shadow: 0 2px 8px rgba(0, 0, 0, 0.08);
        }

        pre {
            background: #0b1220;
            color: #d0dde5;
            padding: 15px;
            border-radius: 6px;
            overflow-x: auto;
        }

        img {
            width: 100%;
            border-radius: 6px;
            margin-top: 10px;
        }

        footer {
            text-align: center;
            padding: 20px;
            margin-top: 40px;
            color: #555;
            font-size: 14px;
        }

        nav {
            background: #ffffff;
            padding: 10px 20px;
            border-bottom: 1px solid #eee;
            position: sticky;
            top: 0;
            z-index: 10;
        }

        nav a {
            margin-right: 15px;
            text-decoration: none;
            color: #0f766e;
            font-weight: bold;
        }

        nav a:hover {
            text-decoration: underline;
        }
    </style>
</head>

<body>

    <header>

        <h3>Technological University Mandalay <br>
            Department of Mechatonics Enginnering
        </h3>
        <h1>Autonomous Waitress Robot Using LIDAR</h1>
        <div style="display: flex; justify-content: space-between;">
            <p align="left">Supervised by; Daw Su Myat Hlaing</p>
            <p align="right">Candidate; Mg Kyi Lwin Han</p>
        </div>
    </header>

    <nav>
        <a href="#abstract">Abstract</a>
        <a href="#intro">Introduction</a>
        <a href="#method">Methodology</a>
        <a href="#results">Results</a>
        <a href="#conclusion">Conclusion</a>
    </nav>

    <div class="container">

        <!-- ABSTRACT -->
        <section id="abstract" class="card">
            <h2>Abstract</h2>
            <p>
                This thesis aims to simulate and construct ROS2-based autonomous waitress robot capable of delivering
                ordered items from web-based menu to respective tables avoiding static and dynamic obstacles in
                real-world environment. The system mainly used Robot Operating System 2 (ROS2) alongside with SLAM
                Toolbox, Nav2 stack for mapping and navigation, and Unified Robot Description Format (URDF), Simulation
                Description Format (SDF) for robot description and simulation in Gazebo. This system works really well
                in simulation and shows acceptable performance in real world application due to environmental and
                sensory-communication effect. However, no image processing method is applied and sometimes slippery on
                uneven and slope surfaces.

            </p>
        </section>

        <!-- INTRODUCTION -->
        <section id="intro" class="card">
            <h2>1. Introduction</h2>
            <p>
                The goal of this thesis is to solve the labor shortage in the restaurant
                industry by implementing a fully autonomous robotic waitress capable of
                navigating, delivering orders, and interacting with web-based ordering
                system.
            </p>

            <h3>Aims & Objectives</h3>
            <p>The aim of this thesis is to implement and control an autonomous waitress
                robot in real world, delivering safely and reliably some meal from kitchen to designated
                tables. The objectives of this system are as follow;</p>
            <ul>
                <li>To design a differential-drive mobile robot platform with Raspberry Pi5
                    and Arduino Uno microcontroller.</li>
                <li>To implement Simultaneous Localization and Mapping (SLAM)
                    algorithm using LiDAR for restaurant mapping.</li>
                <li>To develop autonomous navigation using ROS2 Nav2 stack.</li>
                <li>To simulate and demonstrate the real time operation of the system in Gazebo and Rviz2
                    on PC.</li>
                <li>To construct QR code scannable web-based restaurant order handling
                    system.</li>
                <li>To evaluate robot performance in obstacle avoidance and path-following
                    tasks.</li>
            </ul>
        </section>

        <!-- METHODOLOGY -->
        <section id="method" class="card">
            <h2>2. Methodology</h2>
            <p>This system use Simultaneous Localization and Mapping (SLAM) algorithm for occupancy grid mapping,
                utilizing two wheel position encoders for robot transformations and correcting drifting position errors
                with 360-degree range laser data from RP LIDAR A1. Based on the obtained static map, global and local
                costmap are generated which are later used by A* path planning algorithm to calculate shortest path to
                destination and Dynamic Window Approach (DWA) for following that path generating appropriate velocity
                commands to robot. The dynamic obstacle avoidance is performed by real-time laser-scanned data from
                LIDAR, while delivering ordered dishes to the customers.
            </p>

            <img src="your-diagram.png" alt="System Diagram">

            <h3>Example Robot Task JSON</h3>
            <pre>
            GET /api/robot/task

            {
            "task": "deliver",
            "table": 5,
            "orderId": 10,
            "pose": { "x": 2.3, "y": 1.1, "yaw": 1.57 }
            }
            </pre>
        </section>

        <!-- OVERALL BLOCK DIAGRAM -->
        <section id="overall_blockdiagram" class="card">
            <h2>3. Overall Blockdiagram</h2>
            <p>The overall block diagram is as illustrated follow:
            </p>

            <img src="/home/eq/Pictures/Screenshots/For GitHub AWR/img_0_overall_block_diagram.png" alt="overall_blockdiagram">
            
        </section>

        <!-- OVERALL FLOWCHAT -->
        <section id="overall_flowchart" class="card">
            <h2>4. Overall Flowchart</h2>
            <p>The overall flowchart is as shown below:
            </p>

            <img src="/home/eq/Pictures/Screenshots/For GitHub AWR/img_0_overallk_flowchart.png" style="width: auto; height: auto; align-items: center" alt="System Diagram">
            
        </section>

        <!-- PIN CONNECTIONS -->
        <section id="pin_connection" class="card">
            <h2>5. Pin Connections</h2>
            <p>This system use Simultaneous Localization and Mapping (SLAM) algorithm for occupancy grid mapping,
                utilizing two wheel position encoders for robot transformations and correcting drifting position errors
                with 360-degree range laser data from RP LIDAR A1. Based on the obtained static map, global and local
                costmap are generated which are later used by A* path planning algorithm to calculate shortest path to
                destination and Dynamic Window Approach (DWA) for following that path generating appropriate velocity
                commands to robot. The dynamic obstacle avoidance is performed by real-time laser-scanned data from
                LIDAR, while delivering ordered dishes to the customers.
            </p>

            <img src="your-diagram.png" alt="System Diagram">

            <h3>Example Robot Task JSON</h3>
            
        </section>



        <!-- RESULTS -->
        <section id="results" class="card">
            <h2>6. Test and Results</h2>
            <p>
                The robot successfully navigated multiple table locations using SLAM
                Toolbox and AMCL. Below is an example of the generated 2D map:
            </p>

            <img src="your-map.png" alt="SLAM Map">

            <p>Trajectory and obstacle avoidance visualized in RViz2:</p>
            <img src="your-path.png" alt="Robot Trajectory">
        </section>

        <!-- CONCLUSION -->
        <section id="conclusion" class="card">
            <h2>7. Conclusion</h2>
            <p>
                The autonomous waitress robot demonstrates efficient indoor navigation,
                stable delivery operations, and full integration with a digital ordering
                system. Future work includes multi-robot coordination and enhanced
                computer-vision-based human interaction.
            </p>

            <p><a href="thesis.pdf" style="color:#0f766e;font-weight:bold;" download>Download Full Thesis PDF</a></p>
        </section>

    </div>

    <footer>
        © 2025 Your Name — All Rights Reserved
    </footer>

</body>

</html>