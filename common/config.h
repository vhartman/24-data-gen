#pragma once

namespace manip{
  struct Parameters{
    bool use_early_coll_check_stopping = true;
    bool allow_display = true;

    bool export_images = false;
    bool compress_data = false;

    std::string output_path = "./out/";
  };
};

extern manip::Parameters global_params;