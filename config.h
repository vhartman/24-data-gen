#pragma once

namespace manip{
  struct Parameters{
    bool use_early_coll_check_stopping = true;
    bool allow_display = true;
  };
};

extern manip::Parameters global_params;