#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"
#ifdef USE_REALSENSE
#include "realsense_camera.h"
#endif

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = nullptr;
std::shared_ptr<DepthCamera> FSMState::camera = nullptr;

void init_fsm_state()
{
    auto lowcmd_sub = std::make_shared<unitree::robot::go2::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if(!lowcmd_sub->isTimeout())
    {
        spdlog::critical("The other process is using the lowcmd channel, please close it first.");
        unitree::robot::go2::shutdown();
        // exit(0);
    }
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

void init_camera()
{
    auto cam_cfg = param::config["camera"];
    auto type = cam_cfg ? cam_cfg["type"].as<std::string>("none") : "none";
    if(type == "none") {
        spdlog::info("No depth camera configured.");
        return;
    }

    DepthCamera::Config cfg;
    cfg.out_width = cam_cfg["width"].as<int>(cfg.out_width);
    cfg.out_height = cam_cfg["height"].as<int>(cfg.out_height);
    cfg.out_fovy_deg = cam_cfg["fovy"].as<float>(cfg.out_fovy_deg);

    if(type == "realsense") {
#ifdef USE_REALSENSE
        RealSenseConfig rs;
        rs.stream_width = cam_cfg["stream_width"].as<int>(rs.stream_width);
        rs.stream_height = cam_cfg["stream_height"].as<int>(rs.stream_height);
        rs.fps = cam_cfg["fps"].as<int>(rs.fps);
        rs.serial = cam_cfg["serial"].as<std::string>(rs.serial);
        FSMState::camera = std::make_shared<RealSenseCamera>(rs, cfg);
#else
        spdlog::critical("camera.type is 'realsense' but go2_ctrl was built without librealsense2.");
        std::exit(1);
#endif
    } else {
        spdlog::critical("Unknown camera.type '{}'.", type);
        std::exit(1);
    }

    if(!FSMState::camera->start()) {
        spdlog::critical("Depth camera failed to start.");
        std::exit(1);
    }
    spdlog::info("Depth camera started ({}x{}, fovy {} deg).", cfg.out_width, cfg.out_height, cfg.out_fovy_deg);
}

int main(int argc, char** argv)
{
    // Load parameters
    auto vm = param::helper(argc, argv);

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     Go2 Controller \n";

    // Unitree DDS Config
    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    init_fsm_state();
    init_camera();

    // Initialize FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();

    std::cout << "Press [L2 + up] to enter FixStand mode.\n";
    std::cout << "And then press [R2 + A] to start controlling the robot.\n";

    while (true)
    {
        sleep(1);
    }
    
    return 0;
}

