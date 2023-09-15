#pragma once

arr center(arr pts) { return pts; }

arr scale(arr pts) { return pts; }

std::map<uint, arr>
overlappingCircles(const uint n1 = 25, const uint n2 = 25,
                   const double r1 = 0.05, const double r2 = 0.05,
                   const double x1 = -0.03, const double x2 = 0.03,
                   const double y1 = 0, const double y2 = 0) {
  std::map<uint, arr> res;
  arr pts;

  for (uint i = 0; i < n1; ++i) {
    const arr pt = {x1 + r1 * sin(2. * i / n1 * 3.1415),
                    y1 + r1 * cos(2. * i / n1 * 3.1415)};
    pts.append(pt);
  }
  pts.reshape(-1, 2);
  res[0] = pts;

  pts.clear();
  for (uint i = 0; i < n2; ++i) {
    const arr pt = {x2 + r2 * sin(2. * i / n2 * 3.1415),
                    y2 + r2 * cos(2. * i / n2 * 3.1415)};
    pts.append(pt);
  }
  pts.reshape(-1, 2);
  res[1] = pts;

  return res;
}

arr grid(const uint nx = 5, const uint ny = 5, const double x = 0.3,
         const double y = 0.3) {
  arr pts;
  for (uint i = 0; i < nx; ++i) {
    for (uint j = 0; j < ny; ++j) {
      const arr point = {1. * i / (nx - 1) * x - x / 2,
                         1. * j / (ny - 1) * y - y / 2};
      pts.append(point);
    }
  }

  pts.reshape(-1, 2);
  return pts;
}

arr greedy_counterexample(){
  return grid(2, 2, 0.01, 0.4);
}

arr spiral(const uint n = 20) {
  arr pts;
  for (uint i = 0; i < n; ++i) {
    const arr pt = {0.25 * (i + 1) / n * sin(i * 2 / 3.1415),
                    0.25 * (i + 1) / n * cos(i * 2 / 3.1415)};
    pts.append(pt);
  }

  pts.reshape(-1, 2);
  return pts;
}

arr circles(const double x = 0.25, const uint num = 8) {
  arr pts;
  for (uint j = 0; j < num + 1; ++j) {
    double r = 0.5 * x * (1. - 1. * j * j / (num * num));
    uint n = (num - j) * (num - j) + 1;
    double phi = (rand() % 1000) / 1000.;

    // std::cout << r << " " << n << std::endl;
    for (uint i = 0; i < n; ++i) {
      const arr pt = {r * sin(phi + i * 2 * 3.1415 / n),
                      r * cos(phi + i * 2 * 3.1415 / n)};
      pts.append(pt);
    }
  }

  pts.reshape(-1, 2);
  return pts;
}

arr cube(const uint n = 200) {
  arr v = {1, 1, 1};
  v = v / length(v);

  arr R;
  rotationFromAtoB(R, {0, 0, 1}, v);

  rai::Configuration C;

  arr pts;
  for (uint i = 0; i < n; ++i) {
    arr pt(3, 1);

    rndUniform(pt, -1, 1);

    const uint ind = rand() % 3;
    // const uint sign = rand() % 2;
    const uint sign = 1;

    pt[ind] = sign * 2. - 1;

    // rotate
    pt = R * pt;

    // scale down
    pt = pt * 0.07;

    // project
    const arr pt_proj = {pt[0](0), pt[1](0)};
    pts.append(pt_proj);

    auto *dot = C.addFrame("dot", "");
    dot->setShape(rai::ST_sphere, {0.005});
    dot->setPosition(pt);
    dot->setContact(0.);
  }

  C.watch(true);

  pts.reshape(-1, 2);
  return pts;
}

arr randomPts(const uint n = 20, const double lb = -0.1,
              const double ub = 0.1) {
  arr pts;
  for (uint i = 0; i < n; ++i) {
    arr pt(2);
    rndUniform(pt, lb, ub);
    pts.append(pt);
  }

  pts.reshape(-1, 2);
  return pts;
}

std::map<uint, arr> randomPtsMulti(const uint n1 = 20, const uint n2 = 20) {
  std::map<uint, arr> res;

  res[0] = randomPts(n1, -0.07, 0.05);
  res[1] = randomPts(n2, -0.05, 0.07);

  return res;
}

arr LISlogo(bool large=false) {
  arr pts;
  pts << FILE("./in/lis.txt");

  if(!large){
    pts = pts / 10000.;
  }

  // pts = pts + 0.25;

  if (large){
    pts = pts / 2500.;

    for (uint i=0; i<pts.d0; ++i){
      pts(i, 0) += 0.15;
      pts(i, 1) -= 0.1;
    }
  }

  pts.reshape(-1, 2);

  return pts;
}

std::map<uint, arr> LISlogoMulti() {
  arr pts1;
  pts1 << FILE("./in/lis_multi_1.txt");

  arr pts2;
  pts2 << FILE("./in/lis_multi_2.txt");

  pts1 = pts1 / 4000.;
  pts2 = pts2 / 4000.;

  double x = 0.15;
  double y = -0.1;

  for (uint i = 0; i < pts1.d0; ++i) {
    pts1(i, 0) += x;
    pts1(i, 1) += y;
  }
  for (uint i = 0; i < pts2.d0; ++i) {
    pts2(i, 0) += x;
    pts2(i, 1) += y;
  }

  pts1.reshape(-1, 2);
  pts2.reshape(-1, 2);

  std::map<uint, arr> res;
  res[0] = pts1;
  res[1] = pts2;

  return res;
}

