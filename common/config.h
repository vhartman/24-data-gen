#pragma once

namespace manip{
  struct Parameters{
    bool use_early_coll_check_stopping = true;
    bool allow_display = true;

    bool export_images = false;
    bool compress_data = false;
    bool export_txt_files = false;

    std::string output_path = "./out/";

    bool apply_mode_switch_durations = false;
    uint mode_switch_duration_lb = 5;
    uint mode_switch_duration_ub = 15;
  };
};

extern manip::Parameters global_params;