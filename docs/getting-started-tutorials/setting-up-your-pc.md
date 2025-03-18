# Setting up your PC


The **first step** in getting started on the competition is to get access to a computer with the right system requirements, and set it up with the right operating system, software and environment to run the competition packages.


<!-- This guide helps you setup your computer for running the competition environment locally and developing the code.

You can use local Computer/Laptop or Virtual Machine inside your computer or any cloud platform like [Google GCP](https://cloud.google.com/free){target=_blank}, [Amazon AWS](https://aws.amazon.com/free/){target=_blank}, [Microsoft Azure](https://azure.microsoft.com/en-us/free/){target=_blank}, [Digital Ocean](https://try.digitalocean.com/freetrialoffer/){target=_blank}, etc (All Cloud providers have some free trial plan which you can make use of). -->


## System requirements

You will need a computer that has all (or at least most) of these specifications:
    
- A dedicated [GPU](https://en.wikipedia.org/wiki/Graphics_processing_unit){target=_blank},
    - Nvidia cards tend to work well in Ubuntu
- A CPU that is at least an Intel i5, or equivalent,
- At least 4GB of free disk space,
- At least 8GB of RAM,

## Operating System (OS)

We will be using **[Ubuntu Noble Numbat (24.04)](https://releases.ubuntu.com/noble/){target=_blank}** operating system (OS) which is a flavor of [Linux](https://en.wikipedia.org/wiki/Linux){target=_blank}. We know that some participants may have limited or no experience using Ubuntu, so here is a guide on different ways of setting up an operational Ubuntu Noble Numbat instance for this competition.

If you have a Windows PC (or any other operating system different from Ubuntu Noble Numbat 24.04), here are two (2) options to explore:
<!-- If you have a Windows PC (or any other operating system different from Ubuntu Jammy 22.04), here are three (3) options to explore: -->

- Option 1 (Recommended): **Dual-Boot**: Install **[Ubuntu Noble Numbat (24.04)](https://releases.ubuntu.com/noble/){target=_blank}** in a dual-boot alongside your Windows OS.
<!-- - Option 2: **Using Docker:** Run **[Ubuntu Jammy (22.04)](https://releases.ubuntu.com/jammy/){target=_blank}** in a Docker container on your native Windows OS. -->
- Option 2: **Using Virtual Machine:** Run **[Ubuntu Noble Numbat (24.04)](https://releases.ubuntu.com/noble/){target=_blank}** in a Virtual Machine (VM) on your native Windows OS.

=== "Dual-Boot (Recommended)"
    - This [video](https://www.youtube.com/watch?v=qq-7X8zLP7g){target=_blank} walkthroughs the process of dual-booting **[Ubuntu Noble Numbat (24.04)](https://releases.ubuntu.com/jammy/){target=_blank}** alongside your Windows OS. Following this [guide](https://linuxconfig.org/how-to-install-ubuntu-alongside-windows-11-dual-boot){target=_blank} also achieves the same dual-boot setup.

=== "Using Virtual Machine"
    - Installing VirtualBox on your PC to run Ubuntu is outlined in this [video](https://www.youtube.com/watch?v=kSy3NX3Pe-c){target=_blank}. You can also follow this [guide](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview){target=_blank} to install VirtualBox and run Ubuntu.
  
  <!--     - We have provided a [detailed guide](../getting-started-tutorials/setting-up-with-docker.md) on installing Docker on your PC and setting up the right Docker container to run the entire competition.  -->
<!--     - If you are concerned about the dual-boot option, we recommend considering this option. -->
<!---->
<!-- === "Using Virtual Machine" -->
<!--     - Here is a [good guide](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview){target=_blank} you can follow to install VirtualBox on your PC and run Ubuntu.  -->
<!---->


## Installing ROS
<!-- Uncomment for Docker install -->
<!-- !!! note -->
<!--     If you followed the **Using Docker** option above, please SKIP this step. -->

Once you have a fresh **[Ubuntu Noble Numbat (24.04)](https://releases.ubuntu.com/noble/){target=_blank}** installation, the next step is to install ROS. We are using the [ROS2 Jazzy](https://docs.ros.org/en/jazzy) distribution for this competition. You can install ROS2 Jazzy by following [this guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html){target=_blank}, install the Desktop version, `ros-jazzy-desktop`, and follow the rest of the setup guide.

If you prefer video instructions, you can watch this [YouTube video](https://www.youtube.com/watch?v=oTp2UtHZLH0){target=_blank} (there is an accompanying blog post in the video description).

<!-- ## Next Steps

To set up your workspace, there are two available options. You can either use Docker or manually configure your workspace. If you prefer to configure your workspace manually, follow the instructions provided [here](../getting-started-tutorials/setting-up-your-workspace.md). Alternatively, if you would like to set up your workspace using Docker, follow the instructions provided [here](../getting-started-tutorials/setting-up-your-workspace-using-docker.md). -->
