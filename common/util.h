#pragma once

#include <numeric>
#include "types.h"

#include <KOMO/komo.h>

rai::Animation::AnimationPart make_animation_part(rai::Configuration &C,
                                                  const arr &path,
                                                  const FrameL &frames,
                                                  const uint t_start) {
  rai::Animation::AnimationPart anim;

  StringA frameNames;
  for (auto f : frames) {
    frameNames.append(f->name);
  }

  anim.start = t_start;
  anim.frameIDs = framesToIndices(frames);
  anim.frameNames = frameNames;

  const uint dt = path.d0;
  anim.X.resize(dt, frames.N, 7);

  arr q;
  for (uint i = 0; i < path.d0; ++i) {
    q = path[i];
    C.setJointState(q);
    // C.watch(true);
    anim.X[i] = C.getFrameState(frames);
  }
  return anim;
}

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
  // collect all frames that are collidable
  std::vector<std::string> do_not_delete{"tip", "goal"};

  FrameL allCollidingFrames;
  for (auto f : C.frames) {
    if (f->shape && f->getShape().cont != 0) {
      allCollidingFrames.append(f);
    }
  }

  FrameL joints = C.getJoints();
  std::vector<uint> ids;
  for (const auto f : allCollidingFrames) {
    ids.push_back(f->ID);
    rai::Frame *upwards = f->parent;
    while (upwards) {
      ids.push_back(upwards->ID);
      upwards = upwards->parent;
    }
  }

  FrameL remove;
  for (auto f : C.frames) {
    bool skip = false;
    for (const auto &str: do_not_delete){
      if (f->name.contains(str.c_str())){
        skip = true;
        break;
      }
    }
    if(skip){continue;}
    if (f->getShape().cont == 0 &&
        std::find(ids.begin(), ids.end(), f->ID) == ids.end()) {
      // std::cout << f->name << std::endl;
      remove.append(f);
    }
  }

  for (auto f : remove) {
    delete f;
  }
}

uintA get_cant_collide_pairs(const rai::Configuration &C) {
  uintA cantCollidePairs;
  for (uint i=0; i<C.frames.d0; ++i){
    const auto a = C.frames(i);
    for (uint j=i+1; j<C.frames.d0; ++j){
      const auto b = C.frames(j);

      if (!a->getShape().canCollideWith(b)) {
        cantCollidePairs.append(TUP(a->ID, b->ID));
        // std::cout << a->name << " " << b->name << std::endl;
      }

      // Skip if one of the objects is an object we move.
      // Otherwise some collision with the table will be ignored.
      if (a->name.contains("obj") || b->name.contains("obj")){
        continue;
      }

      if (b == a->parent || a == b->parent) {
        cantCollidePairs.append(TUP(a->ID, b->ID));
        // std::cout << a->name << " " << b->name << std::endl;
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
