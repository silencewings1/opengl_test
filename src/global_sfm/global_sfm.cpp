#include "global_sfm.h"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <openMVG/cameras/cameras.hpp>
#include <openMVG/geodesy/geodesy.hpp>
#include <openMVG/image/image_io.hpp>
#include <openMVG/numeric/eigen_alias_definition.hpp>
#include <openMVG/sfm/sfm_data.hpp>
#include <openMVG/sfm/sfm_data_colorization.hpp>
#include <openMVG/sfm/sfm_data_io.hpp>
#include <openMVG/sfm/sfm_data_utils.hpp>
#include <openMVG/sfm/sfm_view.hpp>
#include <openMVG/sfm/sfm_view_priors.hpp>
#include <openMVG/system/timer.hpp>
#include <openMVG/types.hpp>

#include <openMVG/sfm/pipelines/global/GlobalSfM_rotation_averaging.hpp>
#include <openMVG/sfm/pipelines/global/GlobalSfM_translation_averaging.hpp>
#include <openMVG/sfm/pipelines/global/sfm_global_engine_relative_motions.hpp>
#include <openMVG/sfm/pipelines/sfm_features_provider.hpp>
#include <openMVG/sfm/pipelines/sfm_matches_provider.hpp>
#include <openMVG/sfm/pipelines/sfm_regions_provider.hpp>

#include <openMVG/matching_image_collection/Cascade_Hashing_Matcher_Regions.hpp>
#include <openMVG/matching_image_collection/F_ACRobust.hpp>
#include <openMVG/matching_image_collection/GeometricFilter.hpp>
#include <openMVG/matching_image_collection/Matcher_Regions.hpp>
#include <openMVG/matching_image_collection/Pair_Builder.hpp>

#include <nonFree/sift/SIFT_describer_io.hpp>
#include <third_party/progress/progress_display.hpp>
#include <third_party/stlplus3/filesystemSimplified/file_system.hpp>

#define USE_STEREO 1
// #define USE_IPHONE7P 1
#include "def/cam_para.h"
#include "model_generator/ply/SfMPlyHelper.hpp"

#ifdef USE_STEREO
#include "rectify/rectifier.h"
#endif

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::geodesy;
using namespace openMVG::image;
using namespace openMVG::sfm;
using namespace openMVG::features;
using namespace openMVG::matching;
using namespace openMVG::matching_image_collection;

#include <chrono>
#include <thread>
using namespace std::chrono_literals;

namespace
{

const std::string sfm_data_file = "sfm_data.json";
const std::string color_ply_name = "color.ply";
const std::string left_prefix = "left_";
const std::string right_prefix = "right_";
const std::string rect_prefix = "rect_";
const std::string ref_id = "1";

struct My_Regions_Provider : Regions_Provider
{
public:
    My_Regions_Provider(std::unique_ptr<Regions>& region_type)
        : Regions_Provider()
    {
        region_type_.reset(region_type->EmptyClone());
    }

    void insert(IndexT id_view, std::shared_ptr<Regions> regions)
    {
        cache_[id_view] = regions;
    }
};

/// Export camera poses positions as a Vec3 vector
std::vector<Vec3> GetCameraPositions(const SfM_Data& sfm_data)
{
    std::vector<Vec3> vec_camPosition;

    for (const auto& view : sfm_data.GetViews())
    {
        if (sfm_data.IsPoseAndIntrinsicDefined(view.second.get()))
        {
            const geometry::Pose3 pose = sfm_data.GetPoseOrDie(view.second.get());
            vec_camPosition.push_back(pose.center());
        }
    }

    return vec_camPosition;
}

// get camera position with given left && right path
// rect_left_00.jpg
// rect_right_00.jpg
struct PPP
{
    Vec3 left_pos, right_pos;
    double Distance() const { return (left_pos - right_pos).norm(); }
};

std::vector<Vec3> GetCameraPositionsNew(const SfM_Data& sfm_data, PPP& ppp)
{
    std::vector<Vec3> vec_camPosition;

    for (const auto& view : sfm_data.GetViews())
    {
        View* vv = view.second.get();
        if (sfm_data.IsPoseAndIntrinsicDefined(vv))
        {
            const geometry::Pose3 pose = sfm_data.GetPoseOrDie(vv);
            vec_camPosition.push_back(pose.center());

            if (stlplus::basename_part(vv->s_Img_path) == rect_prefix + left_prefix + ref_id)
            {
                ppp.left_pos = vec_camPosition.back();
            }
            else if (stlplus::basename_part(vv->s_Img_path) == rect_prefix + right_prefix + ref_id)
            {
                ppp.right_pos = vec_camPosition.back();
            }
        }
    }

    return vec_camPosition;
}

void AdjustModelPosition(std::vector<Vec3>& vec_points,
                         std::vector<Vec3>& vec_camPos,
                         const PPP& ppp)
{
    const auto scale = CameraPara::baseline / ppp.Distance();
    std::cout << "left_pos:\t" << ppp.left_pos.transpose() << std::endl;
    std::cout << "right_pos:\t" << ppp.right_pos.transpose() << std::endl;

    for (auto& vec : vec_points)
    {
        vec -= ppp.left_pos;
        vec *= scale;
    }
    for (auto& vec : vec_camPos)
    {
        vec -= ppp.left_pos;
        vec *= scale;
    }
}

} // namespace

GlobalSfM::GlobalSfM(const std::string images_dir,
                     const std::string output_dir)
    : images_dir(images_dir)
    , output_dir(output_dir)
{
}

std::string GlobalSfM::GetModelPath() const
{
    return output_dir + color_ply_name;
}

bool GlobalSfM::Solve() const
{
    openMVG::system::Timer total_timer;
    SfM_Data sfm_data;

    // image list
    {
        if (!stlplus::folder_exists(images_dir))
        {
            std::cerr << "\nThe input directory doesn't exist" << std::endl;
            return false;
        }

        if (!stlplus::folder_exists(output_dir))
        {
            if (!stlplus::folder_create(output_dir))
            {
                std::cerr << "\nCannot create output directory" << std::endl;
                return false;
            }
        }

        std::string images_root_path = images_dir;

#ifdef USE_STEREO
        {
            Rectifier rectifier(cv::Size(1920, 1080), maps_dir);

            const std::string rectified_dir = output_dir + "/rectified";
            images_root_path = rectified_dir;
            if (!stlplus::folder_exists(rectified_dir))
            {
                if (!stlplus::folder_create(rectified_dir))
                {
                    std::cerr << "\nCannot create rectify directory" << std::endl;
                    return false;
                }
            }

            std::vector<std::string> vec_image = stlplus::folder_files(images_dir);
            std::sort(vec_image.begin(), vec_image.end());
            for (const auto& iter_image : vec_image)
            {
                const std::string sImageFilename = stlplus::create_filespec(images_dir, iter_image);
                const std::string sImFilenamePart = stlplus::filename_part(sImageFilename);

                if (openMVG::image::GetFormat(sImageFilename.c_str()) == openMVG::image::Unknown)
                {
                    std::cout << sImFilenamePart << ": Unkown image file format.\n";
                    continue;
                }

                auto side_id = [&sImFilenamePart]() {
                    if (sImFilenamePart.find(left_prefix) != std::string::npos)
                        return Rectifier::ImgIdx::LEFT;
                    if (sImFilenamePart.find(right_prefix) != std::string::npos)
                        return Rectifier::ImgIdx::RIGHT;
                    return Rectifier::ImgIdx::INVAILD;
                }();

                if (side_id == Rectifier::ImgIdx::INVAILD)
                {
                    std::cout << sImFilenamePart << ": not left or right image file.\n";
                    continue;
                }

                cv::Mat img = cv::imread(sImageFilename);
                img = rectifier.rectify(img, side_id);
                cv::imwrite(stlplus::create_filespec(rectified_dir, rect_prefix + sImFilenamePart), img);
            }

            std::this_thread::sleep_for(100ms);
        }
#endif

        sfm_data.s_root_path = images_root_path;
        Views& views = sfm_data.views;
        Intrinsics& intrinsics = sfm_data.intrinsics;

        std::vector<std::string> vec_image = stlplus::folder_files(images_root_path);
        if (vec_image.empty())
        {
            std::cout << "rectified image empty\n";
            return false;
        }

        std::sort(vec_image.begin(), vec_image.end());
        for (const auto& iter_image : vec_image)
        {
            const std::string sImageFilename = stlplus::create_filespec(images_root_path, iter_image);
            const std::string sImFilenamePart = stlplus::filename_part(sImageFilename);

            if (openMVG::image::GetFormat(sImageFilename.c_str()) == openMVG::image::Unknown)
            {
                std::cout << sImFilenamePart << ": Unkown image file format.\n";
                continue;
            }

            ImageHeader imgHeader;
            if (!openMVG::image::ReadImageHeader(sImageFilename.c_str(), &imgHeader))
                continue;

            double width = imgHeader.width;
            double height = imgHeader.height;
            double focal = CameraPara::fx;
            double ppx = CameraPara::cx;
            double ppy = CameraPara::cy;

            View v(iter_image, views.size(), views.size(), views.size(), width, height);
            views[v.id_view] = std::make_shared<View>(v);

            intrinsics[v.id_intrinsic] =
                std::make_shared<Pinhole_Intrinsic_Radial_K3>(width, height, focal, ppx, ppy, 0.0, 0.0, 0.0);
        }

        GroupSharedIntrinsics(sfm_data);

        if (!openMVG::sfm::Save(sfm_data,
                                stlplus::create_filespec(output_dir, sfm_data_file).c_str(),
                                ESfM_Data(VIEWS | INTRINSICS)))
        {
            return false;
        }

        std::cout << std::endl
                  << "SfMInit_ImageListing report:\n"
                  << "listed #File(s): " << vec_image.size() << "\n"
                  << "usable #File(s) listed in sfm_data: " << sfm_data.GetViews().size() << "\n"
                  << "usable #Intrinsic(s) listed in sfm_data: " << sfm_data.GetIntrinsics().size() << std::endl;
    }

    // feature && matches && sfm
    {
        // regions
        std::unique_ptr<Regions> regions_type = std::make_unique<SIFT_Regions>();
        std::shared_ptr<Regions_Provider> regions_provider =
            std::make_shared<My_Regions_Provider>(regions_type);

        // feature
        auto feats_provider = std::make_shared<Features_Provider>();
        {
            system::Timer timer;
            Image<unsigned char> imageGray;
            auto my_regions_provider = std::dynamic_pointer_cast<My_Regions_Provider>(regions_provider);

            C_Progress_display my_progress_bar(sfm_data.GetViews().size(),
                                               std::cout, "\n- EXTRACT FEATURES -\n");

            for (int i = 0; i < static_cast<int>(sfm_data.views.size()); ++i)
            {
                Views::const_iterator iterViews = sfm_data.views.begin();
                std::advance(iterViews, i);
                const View* view = iterViews->second.get();
                const std::string
                    sView_filename = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path);

                if (!ReadImage(sView_filename.c_str(), &imageGray))
                    continue;

                std::unique_ptr<Image_describer> image_describer =
                    std::make_unique<SIFT_Image_describer>();
                auto regions = image_describer->Describe(imageGray, nullptr);
                feats_provider->feats_per_view[view->id_view] = regions->GetRegionsPositions();

                std::unique_ptr<features::Regions> regions_ptr(regions_type->EmptyClone());
                regions_ptr = std::move(regions);
                my_regions_provider->insert(view->id_view, std::move(regions_ptr));

                ++my_progress_bar;
            }

            std::cout << "Task done in (s): " << timer.elapsed() << std::endl;
        }

        // matches
        auto matches_provider = std::make_shared<Matches_Provider>();
        {
            PairWiseMatches map_PutativesMatches;
            Pair_Set pairs = exhaustivePairs(sfm_data.GetViews().size());

            std::unique_ptr<Matcher> collectionMatcher =
                std::make_unique<Cascade_Hashing_Matcher_Regions>(0.8);
            collectionMatcher->Match(regions_provider, pairs, map_PutativesMatches);

            auto filter_ptr = std::make_unique<ImageCollectionGeometricFilter>(
                &sfm_data, regions_provider);
            filter_ptr->Robust_model_estimation(
                GeometricFilter_FMatrix_AC(4.0, 2048),
                map_PutativesMatches, false, 0.6);
            PairWiseMatches map_GeometricMatches = filter_ptr->Get_geometric_matches();

            matches_provider->pairWise_matches_ = map_GeometricMatches;
        }

        // golbal sfm
        {
            GlobalSfMReconstructionEngine_RelativeMotions sfmEngine(sfm_data, output_dir);

            // Configure the features_provider & the matches_provider
            sfmEngine.SetFeaturesProvider(feats_provider.get());
            sfmEngine.SetMatchesProvider(matches_provider.get());

            // Configure reconstruction parameters
            sfmEngine.Set_Intrinsics_Refinement_Type(Intrinsic_Parameter_Type::ADJUST_ALL);
            sfmEngine.Set_Use_Motion_Prior(false);

            // Configure motion averaging method
            sfmEngine.SetRotationAveragingMethod(
                ERotationAveragingMethod::ROTATION_AVERAGING_L2);
            sfmEngine.SetTranslationAveragingMethod(
                ETranslationAveragingMethod::TRANSLATION_AVERAGING_SOFTL1);

            openMVG::system::Timer timer;
            if (!sfmEngine.Process())
            {
                std::cout << "engine process fail\n";
                return false;
            }

            std::cout << " Total Ac-Global-Sfm took (s): " << timer.elapsed() << std::endl;

            openMVG::sfm::Save(sfmEngine.Get_SfM_Data(),
                               stlplus::create_filespec(output_dir, "cloud_and_poses", ".ply"),
                               ESfM_Data(ALL));

            openMVG::sfm::Save(sfmEngine.Get_SfM_Data(),
                               stlplus::create_filespec(output_dir, "sfm_data", ".bin"),
                               ESfM_Data(ALL));

            // color ply
            {
                std::vector<Vec3> vec_3dPoints, vec_tracksColor;
                if (ColorizeTracks(sfmEngine.Get_SfM_Data(), vec_3dPoints, vec_tracksColor))
                {
                    auto vec_camPosition = GetCameraPositions(sfmEngine.Get_SfM_Data());
                    if (!plyHelper::exportToPly(vec_3dPoints, vec_camPosition, output_dir + color_ply_name, &vec_tracksColor))
                    {
                        std::cout << "export " << color_ply_name << " fail\n";
                        return false;
                    }
                }
            }

            // resize_ply
            {
                std::vector<Vec3> vec_3dPoints, vec_tracksColor;
                if (ColorizeTracks(sfmEngine.Get_SfM_Data(), vec_3dPoints, vec_tracksColor))
                {
                    PPP ppp;
                    auto vec_camPosition = GetCameraPositionsNew(sfmEngine.Get_SfM_Data(), ppp);
                    AdjustModelPosition(vec_3dPoints, vec_camPosition, ppp);
                    if (!plyHelper::exportToPly(vec_3dPoints, vec_camPosition, output_dir + "real.ply", &vec_tracksColor))
                    {
                        std::cout << "export "
                                  << "real.ply"
                                  << " fail\n";
                        return false;
                    }
                }
            }
        }
    }

    std::cout << std::endl
              << "--- Total took (s): " << total_timer.elapsed() << " ---\n\n";

    std::this_thread::sleep_for(100ms);

    return true;
}