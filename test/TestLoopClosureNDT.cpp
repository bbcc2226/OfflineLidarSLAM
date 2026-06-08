// TestLoopClosureNDT.cpp
//
// Loads the submap and aligned-scan PLY files saved by VerifyLoopSubmap
// (requires save_loop_closure_debug_info: true in Config.yaml) and re-runs
// the reusable coarse-to-fine scan matching path.
//
// Usage:
//   Set the KF_ID below (or override via the --gtest_filter env), build,
//   then run:
//       ./build/offline_lidar_slam/test_loop_closure_ndt
//
//   Files expected (same paths as submap_publisher.py):
//       /tmp/loop_submap_kf<KF_ID>.ply        — history submap (NDT target)
//       /tmp/loop_curr_aligned_kf<KF_ID>.ply  — current scan   (NDT source)

#include <gtest/gtest.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <limits>
#include <string>

#include "NDT_INC.hpp"
#include "ScanMatching.hpp"
#include "Common.hpp"
#include "DataType.hpp"
#include "TicToc.hpp"

// ---- Configure here -------------------------------------------------------
// Set to the keyframe ID that triggered the loop closure you want to inspect.
// The test will look for /tmp/loop_submap_kf<KF_ID>.ply etc.
static constexpr int KF_ID = 437;
// ---------------------------------------------------------------------------

static std::string submap_path()
{
    return "/tmp/loop_submap_kf" + std::to_string(KF_ID) + ".ply";
}

static std::string raw_scan_path()
{
    return "/tmp/loop_curr_raw_kf" + std::to_string(KF_ID) + ".ply";
}

static std::string init_guess_path()
{
    return "/tmp/loop_init_guess_kf" + std::to_string(KF_ID) + ".txt";
}

static bool LoadInitGuess(Se3& out)
{
    std::ifstream ifs(init_guess_path());
    if (!ifs.is_open()) return false;
    double tx, ty, tz, qw, qx, qy, qz;
    if (!(ifs >> tx >> ty >> tz >> qw >> qx >> qy >> qz)) return false;
    Eigen::Quaterniond q(qw, qx, qy, qz);
    out = Se3(q.normalized(), Eigen::Vector3d(tx, ty, tz));
    return true;
}

// ---------------------------------------------------------------------------
// TestLoopClosureNDT
// Replays loop-closure matching via the reusable CoarseToFineRegistration path.
// ---------------------------------------------------------------------------
TEST(LoopClosureNDT, ScanMatchFromSavedFiles)
{
    // --- Load submap (NDT target) ---
    auto submap_ptr = std::make_shared<PointCloud>();
    ASSERT_TRUE(LoadPLY(submap_path(), submap_ptr))
        << "Could not load submap PLY: " << submap_path()
        << "\n  → Make sure save_loop_closure_debug_info: true in Config.yaml"
        << " and the pipeline has run past frame " << KF_ID;

    // --- Load initial guess ---
    Se3 init_guess;
    bool has_init_guess = LoadInitGuess(init_guess);
    if (has_init_guess) {
        std::cout << "[TestLoopClosureNDT] Loaded init guess: t="
                  << init_guess.translation().transpose() << "\n";
    } else {
        std::cout << "[TestLoopClosureNDT] WARNING: no init guess file at "
                  << init_guess_path() << " — using identity (re-run pipeline to generate)\n";
    }

    // --- Load current scan (NDT source) ---
    // Runtime VerifyLoopSubmap saves the already-filtered local scan to raw_path,
    // so reloading it here should not apply another voxel filter pass.
    // Fall back to the post-alignment scan with identity guess if the raw file is missing.
    auto curr_ptr = std::make_shared<PointCloud>();
    bool use_raw = LoadPLY(raw_scan_path(), curr_ptr) && curr_ptr->pt_list_.size() > 0;
    if (use_raw) {
        std::cout << "[TestLoopClosureNDT] Using raw pre-alignment scan (production replay)\n";
    } else {
        std::cout << "[TestLoopClosureNDT] Raw scan not found, falling back to aligned scan + identity guess\n";
        curr_ptr = std::make_shared<PointCloud>();
        init_guess = Se3();
        ASSERT_TRUE(LoadPLY("/tmp/loop_curr_aligned_kf" + std::to_string(KF_ID) + ".ply", curr_ptr))
            << "Could not load any current scan PLY for kf " << KF_ID;
    }

    std::cout << "[TestLoopClosureNDT] Submap points : " << submap_ptr->pt_list_.size() << "\n";
    std::cout << "[TestLoopClosureNDT] Current points: " << curr_ptr->pt_list_.size()   << "\n";

    // The saved debug PLYs are already the filtered inputs from VerifyLoopSubmap.

    std::cout << "[TestLoopClosureNDT] After downsample — submap: " << submap_ptr->pt_list_.size()
              << "  current: " << curr_ptr->pt_list_.size() << "\n";

    // --- Build NDT map and fine-stage target with the saved submap ---
    NDT_INC ndt(10.0);
    ndt.AddCloud(submap_ptr);

    CoarseToFineRegistration scan_matcher(
        1.0,
        20,
        1e-2,
        1e-2);
    scan_matcher.AddTargetFrame(submap_ptr);

    // --- Align with loaded initial guess (or identity if unavailable) ---
    TicToc timer;
    timer.tic();
    Se3 ndt_result = ndt.Align(curr_ptr, init_guess, 60);
    timer.toc("[TestLoopClosureNDT] NDT alignment time");

    double ndt_score = ndt.ComputeFitnessScore(curr_ptr, ndt_result);
    std::cout << "[TestLoopClosureNDT] NDT fitness score : " << ndt_score << "\n";
    std::cout << "[TestLoopClosureNDT] NDT result translation: "
              << ndt_result.translation().transpose() << "\n";
    std::cout << "[TestLoopClosureNDT] NDT result rotation (deg): "
              << ndt_result.so3().log().norm() * 180.0 / M_PI << "\n";
    std::cout << "[TestLoopClosureNDT] NDT threshold: "
              << ConfigManager::Get().LoopClosure_.loop_closure_fitness_score_threshold << "\n";

    bool ndt_accept = ndt_score < ConfigManager::Get().LoopClosure_.loop_closure_fitness_score_threshold;
    std::cout << "[TestLoopClosureNDT] NDT would " << (ndt_accept ? "PASS" : "FAIL")
              << " the production gate\n";

    timer.tic();
    Se3 final_result = scan_matcher.Align(curr_ptr, ndt, init_guess, 60);
    timer.toc("[TestLoopClosureNDT] Coarse-to-fine alignment time");

    const double final_score = ndt.ComputeFitnessScore(curr_ptr, final_result);
    const Se3 refinement = ndt_result.inverse() * final_result;
    const double refinement_translation = refinement.translation().norm();
    const double refinement_rotation_deg = refinement.so3().log().norm() * 180.0 / M_PI;
    const bool refined = refinement_translation > 1e-6 || refinement_rotation_deg > 1e-6;

    std::cout << "[TestLoopClosureNDT] Final fitness score: " << final_score << "\n";
    std::cout << "[TestLoopClosureNDT] Fine-stage delta translation: "
              << refinement_translation << " m\n";
    std::cout << "[TestLoopClosureNDT] Fine-stage delta rotation: "
              << refinement_rotation_deg << " deg\n";
    std::cout << "[TestLoopClosureNDT] Fine stage was "
              << (refined ? "applied" : "skipped or returned coarse pose") << "\n";

    std::cout << "[TestLoopClosureNDT] Reusable scan-matching path would "
              << (final_score < ConfigManager::Get().LoopClosure_.loop_closure_fitness_score_threshold ? "PASS" : "FAIL")
              << " the loop-closure NDT gate\n";

    // Save the final replay result so you can compare it against production output.
    std::string result_path = "/tmp/loop_ndt_test_result_kf" + std::to_string(KF_ID) + ".ply";
    auto aligned_result = ApplyTransform(final_result, curr_ptr);
    SaveCloud(aligned_result, result_path);
    std::cout << "[TestLoopClosureNDT] Saved replay-aligned scan to " << result_path << "\n";

    SUCCEED();
}
