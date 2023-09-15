#pragma once

std::vector<uint> straightPerm(const uint n) {
  std::vector<uint> indices(n);
  std::iota(std::begin(indices), std::end(indices), 0);

  return indices;
}

std::vector<uint> straightPerm(const std::vector<arr> &qs) {
  std::vector<uint> indices(qs.size());
  std::iota(std::begin(indices), std::end(indices), 0);

  return indices;
}


void delete_unnecessary_frames(rai::Configuration &C){
  FrameL allCollidingFrames;
  for (auto f: C.frames){
    if (f->shape && f->getShape().cont != 0){
      allCollidingFrames.append(f);
    }
  }

  FrameL joints = C.getJoints();
  std::vector<uint> ids;
  for (auto f: allCollidingFrames){
    ids.push_back(f->ID);
    rai::Frame* upwards = f->parent;
    while(upwards){
      ids.push_back(upwards->ID);
      upwards = upwards->parent;
    }
  }

  FrameL remove;
  for(auto f: C.frames){
    if (f->getShape().cont == 0 && std::find(ids.begin(), ids.end(), f->ID) == ids.end()){
      //std::cout << f->name << std::endl;
      remove.append(f);
    }
  }

  for (auto f: remove){
    delete f;
  }
}

uintA get_cant_collide_pairs(const rai::Configuration &C) {
  uintA cantCollidePairs;
  for (uint i=0; i<C.frames.d0; ++i){
    const auto a = C.frames(i);
    for (uint j=i+1; j<C.frames.d0; ++j){
      const auto b = C.frames(j);

      if (!a->getShape().canCollideWith(b)){
        cantCollidePairs.append(TUP(a->ID, b->ID));
      }

      if (b == a->parent || a == b->parent){
        cantCollidePairs.append(TUP(a->ID, b->ID));
      }
    }
  }
  cantCollidePairs.reshape(-1,2);

  return cantCollidePairs;
}

void setKomoToAnimation(KOMO &komo, const rai::Configuration &C,
                        const rai::Animation &A, const arr &ts, int k = -1) {
  CHECK_EQ(ts.d0, komo.timeSlices.d0 - komo.k_order, "wrong komo-size");

  rai::Configuration Ccpy;
  Ccpy.copy(C);

  /*for (uint i=0; i<komo.k_order; ++i){
    A.setToTime(Ccpy, ts(0));

    const FrameL F = komo.timeSlices[i];
    komo.pathConfig.setFrameState(Ccpy.getFrameState(), F);
  }*/

  for (uint i = 0; i < ts.d0; ++i) {
    const uint t = ts(i);
    A.setToTime(Ccpy, t);

    const FrameL F = komo.timeSlices[i + komo.k_order];
    komo.pathConfig.setFrameState(Ccpy.getFrameState(), F);
  }

  // komo.pathConfig.reset_q();
  // komo.pathConfig.ensure_indexedJoints();
  // komo.pathConfig.ensure_q();
}

