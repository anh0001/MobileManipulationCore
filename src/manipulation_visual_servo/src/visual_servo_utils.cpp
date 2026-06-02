// Copyright 2026 MobileManipulationCore Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "manipulation_visual_servo/visual_servo_utils.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include <sensor_msgs/image_encodings.hpp>

namespace manipulation_visual_servo
{

namespace
{

template<typename T>
T clamp_value(T value, T lower, T upper)
{
  return std::max(lower, std::min(upper, value));
}

}  // namespace

bool is_supported_depth_encoding(const std::string & encoding)
{
  return encoding == sensor_msgs::image_encodings::TYPE_16UC1 || encoding == "16UC1";
}

bool decode_depth_image(
  const sensor_msgs::msg::Image & msg,
  cv::Mat & depth_image,
  std::string * error_message)
{
  if (!is_supported_depth_encoding(msg.encoding)) {
    if (error_message != nullptr) {
      *error_message = "unsupported depth encoding '" + msg.encoding + "'";
    }
    return false;
  }

  const std::size_t expected_step = static_cast<std::size_t>(msg.width) * sizeof(uint16_t);
  if (msg.step < expected_step) {
    if (error_message != nullptr) {
      *error_message = "depth step is smaller than width * 2 bytes";
    }
    return false;
  }

  const std::size_t expected_bytes = static_cast<std::size_t>(msg.step) * msg.height;
  if (msg.data.size() < expected_bytes) {
    if (error_message != nullptr) {
      *error_message = "depth payload shorter than expected from step * height";
    }
    return false;
  }

  const cv::Mat view(
    static_cast<int>(msg.height),
    static_cast<int>(msg.width),
    CV_16UC1,
    const_cast<unsigned char *>(msg.data.data()),
    static_cast<std::size_t>(msg.step));
  depth_image = view.clone();
  return true;
}

std::optional<DepthSample> sample_depth_at_roi_anchor(
  const cv::Mat & depth_image,
  const cv::Rect2d & tracked_roi,
  double anchor_x_norm,
  double anchor_y_norm,
  int depth_roi_half_size_px,
  std::size_t min_valid_depth_pixels,
  double max_iqr_m)
{
  if (depth_image.empty() || depth_image.type() != CV_16UC1) {
    return std::nullopt;
  }

  const double clamped_anchor_x = clamp_value(anchor_x_norm, 0.0, 1.0);
  const double clamped_anchor_y = clamp_value(anchor_y_norm, 0.0, 1.0);
  const int cx = clamp_value(
    cvRound(tracked_roi.x + tracked_roi.width * clamped_anchor_x), 0, depth_image.cols - 1);
  const int cy = clamp_value(
    cvRound(tracked_roi.y + tracked_roi.height * clamped_anchor_y), 0, depth_image.rows - 1);
  const int half_size = std::max(0, depth_roi_half_size_px);

  const int x0 = clamp_value(cx - half_size, 0, depth_image.cols - 1);
  const int y0 = clamp_value(cy - half_size, 0, depth_image.rows - 1);
  const int x1 = clamp_value(cx + half_size + 1, x0 + 1, depth_image.cols);
  const int y1 = clamp_value(cy + half_size + 1, y0 + 1, depth_image.rows);
  const cv::Rect sample_roi(x0, y0, x1 - x0, y1 - y0);

  std::vector<uint16_t> valid_depth_mm;
  valid_depth_mm.reserve(static_cast<std::size_t>(sample_roi.area()));
  for (int row = sample_roi.y; row < sample_roi.y + sample_roi.height; ++row) {
    const auto * row_ptr = depth_image.ptr<uint16_t>(row);
    for (int col = sample_roi.x; col < sample_roi.x + sample_roi.width; ++col) {
      const uint16_t depth_mm = row_ptr[col];
      if (depth_mm > 0U) {
        valid_depth_mm.push_back(depth_mm);
      }
    }
  }

  if (valid_depth_mm.size() < min_valid_depth_pixels) {
    return std::nullopt;
  }

  std::sort(valid_depth_mm.begin(), valid_depth_mm.end());
  auto percentile_mm = [&valid_depth_mm](double fraction) {
      if (valid_depth_mm.empty()) {
        return 0.0;
      }

      const double position = clamp_value(fraction, 0.0, 1.0) *
        static_cast<double>(valid_depth_mm.size() - 1);
      const auto lower_index = static_cast<std::size_t>(std::floor(position));
      const auto upper_index = static_cast<std::size_t>(std::ceil(position));
      const double lower_value = static_cast<double>(valid_depth_mm[lower_index]);
      const double upper_value = static_cast<double>(valid_depth_mm[upper_index]);
      const double weight = position - static_cast<double>(lower_index);
      return lower_value + (upper_value - lower_value) * weight;
    };
  const double depth_iqr_m = (percentile_mm(0.75) - percentile_mm(0.25)) / 1000.0;
  if (max_iqr_m > 0.0 && depth_iqr_m > max_iqr_m) {
    return std::nullopt;
  }

  DepthSample sample;
  sample.depth_m = percentile_mm(0.5) / 1000.0;
  sample.valid_pixels = valid_depth_mm.size();
  sample.sampled_roi = sample_roi;
  sample.anchor_px = cx;
  sample.anchor_py = cy;
  sample.depth_iqr_m = depth_iqr_m;
  return sample;
}

CenteringUpdate update_centering_streak(int current_streak, bool centered, int required_cycles)
{
  CenteringUpdate update;
  update.streak = centered ? current_streak + 1 : 0;
  update.stable = update.streak >= std::max(1, required_cycles);
  return update;
}

bool depth_within_standoff(double depth_m, double grasp_standoff_m, double depth_tolerance_m)
{
  return std::abs(depth_m - grasp_standoff_m) <= std::max(0.0, depth_tolerance_m);
}

bool depth_progress_stalled(double oldest_depth_m, double newest_depth_m, double min_progress_m)
{
  return (oldest_depth_m - newest_depth_m) < std::max(0.0, min_progress_m);
}

double compute_depth_velocity_mps(
  double depth_m,
  double grasp_standoff_m,
  double lambda_z,
  double max_linear_velocity)
{
  const double unclamped = lambda_z * (depth_m - grasp_standoff_m);
  const double limit = std::max(0.0, max_linear_velocity);
  if (limit <= 0.0) {
    return 0.0;
  }
  return clamp_value(unclamped, -limit, limit);
}

double project_translation_onto_axis(
  const CartesianVector & translation,
  const CartesianVector & axis)
{
  const double axis_norm = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
  if (axis_norm < 1e-9) {
    return 0.0;
  }

  const double unit_x = axis.x / axis_norm;
  const double unit_y = axis.y / axis_norm;
  const double unit_z = axis.z / axis_norm;
  return translation.x * unit_x + translation.y * unit_y + translation.z * unit_z;
}

TableGraspEstimate estimate_table_grasp(
  const cv::Mat & depth_image_mm,
  const cv::Rect2d & bbox,
  double fx, double fy, double cx, double cy,
  double annulus_margin_frac,
  double min_depth_m, double max_depth_m,
  std::size_t min_plane_points,
  double max_plane_rms_m,
  const cv::Mat & object_mask,
  double gripper_max_opening_m)
{
  TableGraspEstimate result;
  if (depth_image_mm.empty() || depth_image_mm.type() != CV_16UC1 ||
    fx <= 1e-6 || fy <= 1e-6 || bbox.width <= 1.0 || bbox.height <= 1.0)
  {
    return result;
  }

  const int cols = depth_image_mm.cols;
  const int rows = depth_image_mm.rows;

  // Annulus bounds: an outer box expanded by annulus_margin_frac of the bbox
  // size, with the bbox interior excluded.
  const double mx = std::max(4.0, bbox.width * annulus_margin_frac);
  const double my = std::max(4.0, bbox.height * annulus_margin_frac);
  const int ox0 = clamp_value(static_cast<int>(std::floor(bbox.x - mx)), 0, cols - 1);
  const int oy0 = clamp_value(static_cast<int>(std::floor(bbox.y - my)), 0, rows - 1);
  const int ox1 = clamp_value(static_cast<int>(std::ceil(bbox.x + bbox.width + mx)), 0, cols);
  const int oy1 = clamp_value(static_cast<int>(std::ceil(bbox.y + bbox.height + my)), 0, rows);

  const int ix0 = static_cast<int>(std::floor(bbox.x));
  const int iy0 = static_cast<int>(std::floor(bbox.y));
  const int ix1 = static_cast<int>(std::ceil(bbox.x + bbox.width));
  const int iy1 = static_cast<int>(std::ceil(bbox.y + bbox.height));

  // Subsample to keep the plane fit cheap on large ROIs.
  const int step = std::max(1, (ox1 - ox0) / 80);

  std::vector<cv::Point3d> pts;
  pts.reserve(1024);
  double sx = 0.0, sy = 0.0, sz = 0.0;
  for (int v = oy0; v < oy1; v += step) {
    const auto * row_ptr = depth_image_mm.ptr<uint16_t>(v);
    const bool inside_rows = (v >= iy0 && v < iy1);
    for (int u = ox0; u < ox1; u += step) {
      if (inside_rows && u >= ix0 && u < ix1) {
        continue;  // skip object interior
      }
      const uint16_t d_mm = row_ptr[u];
      if (d_mm == 0U) {
        continue;
      }
      const double z = static_cast<double>(d_mm) * 0.001;
      if (z < min_depth_m || z > max_depth_m) {
        continue;
      }
      const double x = (static_cast<double>(u) - cx) * z / fx;
      const double y = (static_cast<double>(v) - cy) * z / fy;
      pts.emplace_back(x, y, z);
      sx += x;
      sy += y;
      sz += z;
    }
  }

  result.plane_points = pts.size();  // report for diagnostics even on failure
  if (pts.size() < min_plane_points) {
    return result;
  }

  // Least-squares plane fit (PCA): centroid + normal (smallest-eigenvalue
  // eigenvector of the point covariance) + RMS residual, for one point set.
  auto fit_plane = [](const std::vector<cv::Point3d> & p,
      cv::Point3d & c_out, cv::Vec3d & n_out, double & rms_out) -> bool {
      const double m = static_cast<double>(p.size());
      if (m < 3.0) {return false;}
      double ax = 0, ay = 0, az = 0;
      for (const auto & q : p) {ax += q.x; ay += q.y; az += q.z;}
      const cv::Point3d c(ax / m, ay / m, az / m);
      double cxx = 0, cyy = 0, czz = 0, cxy = 0, cxz = 0, cyz = 0;
      for (const auto & q : p) {
        const double dx = q.x - c.x, dy = q.y - c.y, dz = q.z - c.z;
        cxx += dx * dx; cyy += dy * dy; czz += dz * dz;
        cxy += dx * dy; cxz += dx * dz; cyz += dy * dz;
      }
      cv::Mat cov = (cv::Mat_<double>(3, 3) <<
        cxx, cxy, cxz, cxy, cyy, cyz, cxz, cyz, czz);
      cv::Mat eval, evec;
      cv::eigen(cov, eval, evec);  // eigenvalues descending; rows = eigenvectors
      cv::Vec3d nrm(evec.at<double>(2, 0), evec.at<double>(2, 1), evec.at<double>(2, 2));
      const double nn = std::sqrt(nrm.dot(nrm));
      if (nn < 1e-9) {return false;}
      nrm *= (1.0 / nn);
      // Orient toward the camera (origin): centroid is in front (z>0).
      if (nrm.dot(cv::Vec3d(c.x, c.y, c.z)) > 0.0) {nrm = -nrm;}
      double sse = 0.0;
      for (const auto & q : p) {
        const double r = nrm[0] * (q.x - c.x) + nrm[1] * (q.y - c.y) + nrm[2] * (q.z - c.z);
        sse += r * r;
      }
      c_out = c; n_out = nrm; rms_out = std::sqrt(sse / m);
      return true;
    };

  // Robust plane via RANSAC. The annulus around a detection bbox can contain a
  // SECOND surface — the table edge, the floor beyond it, or a neighbouring
  // object — plus D405 depth spikes. A least-squares or trimmed fit averages
  // those in (RMS ~0.02 m, normal tilted), which throws the grasp height off by
  // centimetres. RANSAC instead locks onto the largest coplanar consensus (the
  // table), then least-squares refits on just those inliers.
  cv::Point3d centroid;
  cv::Vec3d normal;
  double rms = 0.0;
  {
    const double inlier_thresh = 0.006;  // m — table flatness tolerance
    std::mt19937 rng(20240601u);         // fixed seed: deterministic estimate
    std::uniform_int_distribution<std::size_t> pick(0, pts.size() - 1);
    std::vector<cv::Point3d> best;
    for (int iter = 0; iter < 150; ++iter) {
      const cv::Point3d & a = pts[pick(rng)];
      const cv::Point3d & b = pts[pick(rng)];
      const cv::Point3d & c = pts[pick(rng)];
      cv::Vec3d ab(b.x - a.x, b.y - a.y, b.z - a.z);
      cv::Vec3d ac(c.x - a.x, c.y - a.y, c.z - a.z);
      cv::Vec3d nrm = ab.cross(ac);
      const double nn = std::sqrt(nrm.dot(nrm));
      if (nn < 1e-9) {continue;}
      nrm *= (1.0 / nn);
      std::vector<cv::Point3d> inl;
      inl.reserve(pts.size());
      for (const auto & q : pts) {
        const double r = std::fabs(
          nrm[0] * (q.x - a.x) + nrm[1] * (q.y - a.y) + nrm[2] * (q.z - a.z));
        if (r <= inlier_thresh) {inl.push_back(q);}
      }
      if (inl.size() > best.size()) {best.swap(inl);}
    }
    result.plane_points = best.size();  // report inlier count for diagnostics
    if (best.size() < min_plane_points) {
      return result;
    }
    if (!fit_plane(best, centroid, normal, rms)) {
      return result;
    }
  }

  result.plane_rms_m = rms;  // report for diagnostics even on failure
  if (rms > max_plane_rms_m) {
    return result;
  }

  const cv::Vec3d centroid_v(centroid.x, centroid.y, centroid.z);

  // Primary footprint: the object's OWN points. Deproject valid depth inside a
  // shrunk bbox, keep points standing ABOVE the table plane (an upright object),
  // project them straight down onto the plane and take the robust median. This
  // avoids the grazing-angle projection error of the bbox-bottom ray and is far
  // more accurate for the object's true footprint XY.
  const double OBJ_MIN_H = 0.008;   // m above the table to count as object
  const double OBJ_MAX_H = 0.300;   // m above the table (ignore tall background)
  const std::size_t OBJ_MIN_PTS = 15;

  // Object segmentation: prefer a SAM mask (pixel-accurate, gates exactly the
  // object) over the legacy shrunk-bbox heuristic. With a mask we scan the full
  // bbox and keep only masked pixels; without one we shrink the bbox to avoid
  // mixing in table/background edges.
  const bool use_mask = !object_mask.empty() &&
    object_mask.rows == depth_image_mm.rows && object_mask.cols == depth_image_mm.cols;
  int sx0, sx1, sy0, sy1;
  if (use_mask) {
    sx0 = clamp_value(static_cast<int>(std::floor(bbox.x)), 0, cols - 1);
    sx1 = clamp_value(static_cast<int>(std::ceil(bbox.x + bbox.width)), 0, cols);
    sy0 = clamp_value(static_cast<int>(std::floor(bbox.y)), 0, rows - 1);
    sy1 = clamp_value(static_cast<int>(std::ceil(bbox.y + bbox.height)), 0, rows);
  } else {
    const double shrink = 0.15;     // shrink bbox to avoid edge/background mixing
    sx0 = clamp_value(static_cast<int>(std::floor(bbox.x + bbox.width * shrink)), 0, cols - 1);
    sx1 = clamp_value(static_cast<int>(std::ceil(bbox.x + bbox.width * (1.0 - shrink))), 0, cols);
    sy0 = clamp_value(static_cast<int>(std::floor(bbox.y + bbox.height * shrink)), 0, rows - 1);
    sy1 = clamp_value(static_cast<int>(std::ceil(bbox.y + bbox.height * (1.0 - shrink))), 0, rows);
  }
  const int istep = std::max(1, (sx1 - sx0) / 80);

  std::vector<double> px_, py_, pz_, ph_;
  for (int v = sy0; v < sy1; v += istep) {
    const auto * row_ptr = depth_image_mm.ptr<uint16_t>(v);
    const uint8_t * mask_row = use_mask ? object_mask.ptr<uint8_t>(v) : nullptr;
    for (int u = sx0; u < sx1; u += istep) {
      if (use_mask && mask_row[u] == 0U) {
        continue;
      }
      const uint16_t d_mm = row_ptr[u];
      if (d_mm == 0U) {
        continue;
      }
      const double z = static_cast<double>(d_mm) * 0.001;
      if (z < min_depth_m || z > max_depth_m) {
        continue;
      }
      const cv::Vec3d p((static_cast<double>(u) - cx) * z / fx,
        (static_cast<double>(v) - cy) * z / fy, z);
      const double h = normal.dot(p - centroid_v);  // height above plane (up +)
      if (h < OBJ_MIN_H || h > OBJ_MAX_H) {
        continue;
      }
      const cv::Vec3d proj = p - normal * h;  // drop straight onto the table
      px_.push_back(proj[0]);
      py_.push_back(proj[1]);
      pz_.push_back(proj[2]);
      ph_.push_back(h);
    }
  }
  result.mask_used = use_mask;

  // Graspable-band selector: bin object points by height; per band, PCA the
  // on-plane XY and measure the minor-axis span (the jaw must close across it).
  // Pick a band whose span fits the gripper, preferring mid-body height — this
  // generalises a narrow bottle neck (upper band) and a solid loaf (mid band)
  // without a per-object "neck offset". Writes grasp_height_m / object_width_m.
  auto select_grasp_band = [&](double obj_h) {
      const double band = 0.02;                       // 2 cm height bins
      const double margin = 0.012;                    // jaw clearance
      const double max_w = gripper_max_opening_m - margin;
      double best_h = 0.0, best_w = 0.0;
      double best_cost = 1e9;
      // Bands across the object height. A short object (flat loaf / roll, under
      // ~1.5 bands tall) is handled as ONE band spanning its full height so it
      // is still graspable; taller objects are sliced into 2 cm bands.
      std::vector<std::pair<double, double>> ranges;
      if (obj_h - OBJ_MIN_H < 1.5 * band) {
        ranges.emplace_back(OBJ_MIN_H, std::max(obj_h, OBJ_MIN_H + 0.006));
      } else {
        for (double lo = OBJ_MIN_H; lo + band <= obj_h + 1e-6; lo += band) {
          ranges.emplace_back(lo, lo + band);
        }
      }
      for (const auto & rg : ranges) {
        const double lo = rg.first, hi = rg.second;
        std::vector<double> bx, by;
        for (std::size_t i = 0; i < ph_.size(); ++i) {
          if (ph_[i] >= lo && ph_[i] < hi) {bx.push_back(px_[i]); by.push_back(py_[i]);}
        }
        if (bx.size() < 8) {continue;}
        double mx = 0, my = 0;
        for (std::size_t i = 0; i < bx.size(); ++i) {mx += bx[i]; my += by[i];}
        mx /= bx.size(); my /= by.size();
        double a = 0, b = 0, c = 0;
        for (std::size_t i = 0; i < bx.size(); ++i) {
          const double dx = bx[i] - mx, dy = by[i] - my;
          a += dx * dx; b += dx * dy; c += dy * dy;
        }
        const double nb = static_cast<double>(bx.size());
        a /= nb; b /= nb; c /= nb;
        // Minor eigenvector of the 2x2 covariance [[a,b],[b,c]].
        const double tr = a + c, det = a * c - b * b;
        const double disc = std::sqrt(std::max(0.0, tr * tr * 0.25 - det));
        const double lmin = tr * 0.5 - disc;
        cv::Vec2d minor((std::abs(b) > 1e-12) ? (lmin - c) : 1.0,
          (std::abs(b) > 1e-12) ? b : 0.0);
        const double mn = std::sqrt(minor[0] * minor[0] + minor[1] * minor[1]);
        if (mn < 1e-9) {continue;}
        minor *= (1.0 / mn);
        // Span of the band along the minor axis (5..95 pct to reject stragglers).
        std::vector<double> proj;
        proj.reserve(bx.size());
        for (std::size_t i = 0; i < bx.size(); ++i) {
          proj.push_back((bx[i] - mx) * minor[0] + (by[i] - my) * minor[1]);
        }
        std::sort(proj.begin(), proj.end());
        const double p05 = proj[static_cast<std::size_t>(proj.size() * 0.05)];
        const double p95 = proj[static_cast<std::size_t>(proj.size() * 0.95)];
        const double width = p95 - p05;
        const double mid = 0.5 * obj_h;
        const double band_h = 0.5 * (lo + hi);
        // Cost: feasible bands ranked by closeness to mid-body; infeasible bands
        // heavily penalised but still selectable (narrowest) as a last resort.
        const double feas_pen = (width <= max_w) ? 0.0 : 100.0 * (width - max_w);
        const double cost = feas_pen + std::abs(band_h - mid);
        if (cost < best_cost) {best_cost = cost; best_h = band_h; best_w = width;}
      }
      result.grasp_height_m = best_h;
      result.object_width_m = best_w;
    };

  auto median = [](std::vector<double> & v) {
      std::sort(v.begin(), v.end());
      return v[v.size() / 2];
    };

  // Object top height from the RGB detection bbox top edge: the true top sits
  // directly above the footprint along the table normal, at the height whose
  // projection lands on the bbox top row. Solve
  //   (bbox.y - cy)/fy = (Yf + ny*H) / (Zf + nz*H)   for H.
  // RGB sees the whole object, so this works even when the upper body is
  // translucent and returns no depth (where the depth-based height fails).
  auto bbox_top_height = [&](const cv::Vec3d & fp_cam) -> double {
      const double k = (bbox.y - cy) / fy;
      const double denom_h = k * normal[2] - normal[1];
      if (std::abs(denom_h) < 1e-6) {return -1.0;}
      return (fp_cam[1] - k * fp_cam[2]) / denom_h;
    };

  if (px_.size() >= OBJ_MIN_PTS) {
    // median() sorts in place, so copy the footprint coords (keep px_/py_/ph_
    // index-aligned for the band selector below).
    std::vector<double> mx_ = px_, my_ = py_, mz_ = pz_;
    const cv::Vec3d fp(median(mx_), median(my_), median(mz_));
    // Prefer the RGB-bbox top height; fall back to the 90th-percentile of the
    // object-point heights (depth) only if the bbox solve is out of range.
    std::vector<double> ph_sorted = ph_;
    std::sort(ph_sorted.begin(), ph_sorted.end());
    double top_h = ph_sorted[static_cast<std::size_t>(ph_sorted.size() * 0.9)];
    const double h_bbox = bbox_top_height(fp);
    if (h_bbox > OBJ_MIN_H && h_bbox < OBJ_MAX_H) {
      top_h = h_bbox;
    }
    select_grasp_band(top_h);  // sets result.grasp_height_m / object_width_m
    result.footprint_cam = CartesianVector{fp[0], fp[1], fp[2]};
    result.up_cam = CartesianVector{normal[0], normal[1], normal[2]};
    result.plane_points = pts.size();
    result.plane_rms_m = rms;
    result.footprint_depth_m = fp[2];
    result.object_height_m = top_h;
    result.object_points = px_.size();
    result.valid = true;
    return result;
  }

  // Fallback: ray through the bbox bottom-center pixel, intersected with the plane
  // (used when the object surface gives too few valid depth points).
  const double u_b = bbox.x + bbox.width * 0.5;
  const double v_b = bbox.y + bbox.height;
  const cv::Vec3d dir((u_b - cx) / fx, (v_b - cy) / fy, 1.0);
  const double denom = normal.dot(dir);
  if (std::abs(denom) < 1e-9) {
    return result;
  }
  const double t = normal.dot(centroid_v) / denom;
  if (t < min_depth_m || t > max_depth_m) {
    return result;
  }

  const cv::Vec3d fp_ray(dir[0] * t, dir[1] * t, dir[2] * t);
  const double h_bbox_ray = bbox_top_height(fp_ray);
  result.footprint_cam = CartesianVector{fp_ray[0], fp_ray[1], fp_ray[2]};
  result.up_cam = CartesianVector{normal[0], normal[1], normal[2]};
  result.plane_points = pts.size();
  result.plane_rms_m = rms;
  result.footprint_depth_m = fp_ray[2];
  result.object_height_m =
    (h_bbox_ray > OBJ_MIN_H && h_bbox_ray < OBJ_MAX_H) ? h_bbox_ray : 0.0;
  result.object_points = 0;
  result.valid = true;
  return result;
}

}  // namespace manipulation_visual_servo
