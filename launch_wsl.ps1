param(
    [switch]$Setup,
    [switch]$Teleop,
    [switch]$SimOnly,
    [string]$Mode = "wave"
)

# Convertir ruta de Windows a ruta de WSL (/mnt/d/proyectAgy/mujoco)
$currentDir = (Get-Location).Path.Replace('\', '/').Replace('D:', '/mnt/d').Replace('d:', '/mnt/d').Replace('C:', '/mnt/c').Replace('c:', '/mnt/c')

if ($Setup) {
    Write-Host "Ejecutando setup_wsl2.sh dentro de Ubuntu WSL..." -ForegroundColor Cyan
    wsl -d Ubuntu -e bash -c "cd $currentDir && ./setup_wsl2.sh"
    exit $LASTEXITCODE
}

if ($SimOnly) {
    Write-Host "Iniciando simulacion MuJoCo de Unitree G1 (esperando comandos por DDS)..." -ForegroundColor Green
    wsl -d Ubuntu -e bash -c "cd $currentDir && source /opt/ros/*/setup.bash 2>/dev/null && source install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=file://$currentDir/config/cyclonedds.xml && export ROS_DOMAIN_ID=42 && ros2 launch mujoco_ros2_sim sim.launch.py"
    exit $LASTEXITCODE
}

if ($Teleop) {
    Write-Host "Iniciando consola Teleop de Unitree G1 sobre DDS..." -ForegroundColor Cyan
    wsl -d Ubuntu -e bash -c "cd $currentDir && source /opt/ros/*/setup.bash 2>/dev/null && source install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=file://$currentDir/config/cyclonedds.xml && export ROS_DOMAIN_ID=42 && ros2 run mujoco_ros2_sim teleop_node"
    exit $LASTEXITCODE
}

Write-Host "Lanzando simulacion de Unitree G1 en WSL2 (Modo: $Mode)..." -ForegroundColor Green
wsl -d Ubuntu -e bash -c "cd $currentDir && source /opt/ros/*/setup.bash 2>/dev/null && source install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=file://$currentDir/config/cyclonedds.xml && export ROS_DOMAIN_ID=42 && ros2 launch mujoco_ros2_sim full_demo.launch.py mode:=$Mode"
