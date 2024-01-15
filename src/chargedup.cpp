#include <iostream>
#include <csignal>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <vector>
#include <array>
#include <tuple>
#include <span>
#include <string.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <fmt/format.h>

#include <cscore_cv.h>
#include <cscore_cpp.h>
#include <wpi/json.h>
#include <wpi/raw_ostream.h>
#include <wpi/raw_istream.h>
#include <wpi/StringExtras.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Pose3d.h>
#include <frc/apriltag/AprilTagFields.h>
#include <networktables/NetworkTable.h>
#include <networktables/IntegerTopic.h>
#include <networktables/FloatArrayTopic.h>
#include <networktables/DoubleArrayTopic.h>

#include <core/neon.h>
#include <core/calib.h>
#include <core/vision.h>

#include <cpp-tools/src/sighandle.h>
#include <cpp-tools/src/unix/stats2.h>
#include <cpp-tools/src/unix/stats2.cpp>

#include "field.h"


/* Overclocking: https://www.raspberrypi.com/documentation/computers/config_txt.html#overclocking-options */
/* Ex.
	arm_freq = 2100
	over_voltage = 6
*/

#define DLOG(x) std::cout << (x) << std::endl;
#ifndef DEBUG
#define DEBUG 2
#endif

#ifndef FRC_CONFIG
#define FRC_CONFIG "/boot/frc.json"
#endif
#ifndef NT_IDENTITY
#define NT_IDENTITY "Vision RPI"
#endif
#ifndef SIM_ADDR
#define SIM_ADDR "192.168.0.8"
#endif
#ifndef NT_DEFAULT
#define NT_DEFAULT 0
#endif
#ifndef ENABLE_CAMERASERVER
#define ENABLE_CAMERASERVER 0
#endif
#ifndef PATCH_LIFECAM_V4L_PROP_IDS
#define PATCH_LIFECAM_V4L_PROP_IDS 0
#endif
#ifndef VPIPE_CONFIG_NT_OPTIONS
#define VPIPE_CONFIG_NT_OPTIONS 0
#endif
#ifndef ESTIMATE_TAPE_DISTANCES
#define ESTIMATE_TAPE_DISTANCES 1
#endif

#if ENABLE_CAMERASERVER > 0
#include <cameraserver/CameraServer.h>
#endif
#if VPIPE_CONFIG_NT_OPTIONS > 0
#include <networktables/FloatTopic.h>
#endif

using namespace std::chrono_literals;
using namespace std::chrono;




namespace Constant {
	enum CamID {
		UNDER_ARM_CAM = 0,
		UPPER_CAM = 1,

		NUM_CAMERAS
	};
	static const nt::PubSubOptions
		NT_OPTIONS = { .periodic = 1.0 / 30.0 },
		NT_DATAFLOW_OPTIONS = { .periodic = 1.0 / 30.0, .sendAll = true, .keepDuplicates = true };
	static const std::array<const char*, (size_t)CamID::NUM_CAMERAS>
		CAMERA_TAGS{ "underarm", "upper" };
	static const cs::VideoMode
		DEFAULT_VMODE{ cs::VideoMode::kMJPEG, 640, 480, 30 };
	static constexpr int
		DEFAULT_EXPOSURE = (PATCH_LIFECAM_V4L_PROP_IDS > 0 ? 0 : 40),
		DEFAULT_WBALANCE = -1,
		DEFAULT_BRIGHTNESS = 50,
		DEFAULT_DOWNSCALE = 4;
	static const CalibList
		STATIC_CALIBRATIONS{
			{
				{ "Microsoft® LifeCam HD-3000", {
					{ cv::Size{640, 480}, {
						cv::Mat1f{ {3, 3}, {
							673.6653136395231, 0, 339.861572657799,
							0, 666.1104961259615, 244.21065776461745,
							0, 0, 1
						} },
						cv::Mat1f{ {1, 5}, {
							0.04009256446529976, -0.4529245799337021,
							-0.001655316303789686, -0.00019284071985319236,
							0.5736326357832554
						} }
					} },
					{ cv::Size{1280, 720}, {
						cv::Mat1f{ {3, 3}, {
							1.0990191649538529e+03, 0, 6.2139182601803475e+02,
							0, 1.0952324445233039e+03, 3.5986298825678898e+02,
							0, 0, 1
						} },
						cv::Mat1f{ {1, 5}, {
							1.5547098669227585e-01, -1.3752756798809054e+00,
							9.5749479935394190e-04, 5.4056952639896377e-04,
							2.4270632150764744e+00
						} }
					} }
				} }
			}
		};
	static const cv::Mat_<float>
		DEFAULT_CAM_MATX{cv::Mat_<float>::zeros(3, 3)},
		DEFAULT_CAM_DIST{cv::Mat_<float>::zeros(1, 5)};



	enum PerfMode {
		STREAM_ONLY =		-1,
		NONE =				0,
		RAW_MULTISTREAM =	1,
		SINGLESTREAM =		2,
		RAW_SINGLESTREAM =	3,
		VPP_ONLY =			4,
		APRIL_ONLY =		5,
		RETRO_ONLY =		6
	};

	namespace VRetro {
		/* Tuning Notes:
		 * 10-20 for exposure, 10 is good when the light is mounted correctly, 20 is better for tuning when it is just haning at an indirect angle
		 * 7/8 bits for alpha (red), 6/8 bits for beta (blue) works the best
		 * 25~30 to 100 thresh for 20exp, 20 to 80 thresh for 10exp -- upper bounds for each yields tighter bboxes but may be unstable at farther distance?
		 * Filter upright when camera is aligned with horizontal
		 * Decimate by 4 unless source image is unreasonably small
		 * Ideal ratio is ~0.4125 -- 0.2 to 0.5 seems to work, the main source of variability is the lack of illumination of the edges of the poles, so ratio will dip occationally
		 * 0.6-0.65 for fill thresh is usually good
		 * 60 pixels min -- might want to scale by source size?
		*/
		static constexpr vs2::BGR
			DETECTION_BASE = vs2::BGR::GREEN;
		static constexpr uint8_t
			WST_ALPHA = 0b01111111,
			WST_BETA = 0b00111111,
			WST_THRESH = 35U,
			SRC_EXPOSURE = 10U;
		static constexpr bool
			FILTER_UPRIGHT = true;			// only look for upright rectangles -- use unless the camera is intentionally tilted
		static constexpr float
			DECIMATE_FACTOR = 4.f,			// downscale the source frame by this amount
			TAPE_LOWER_WH_RATIO = 0.20f,	// >>
			TAPE_UPPER_WH_RATIO = 0.50f,	// the tape *should* be 1.66in wide x 4in tall -- ratio: 0.4125
			TAPE_RECT_FILL_THRESH = 0.6f,	// the contour area must be at least this proportion of the bounding rect's area
			TAPE_PIXEL_COUNT_THRESH = 60.f,
			
			TAPE_HEIGHT_INCHES = 4.f;
	}

}






#if ENABLE_CAMERASERVER > 0
class VideoSinkImpl : public cs::VideoSink {
public:
	inline VideoSinkImpl(CS_Sink h) : VideoSink(h) {}	// this constructor is protected so we have to subclass to use it publicly
};
#endif

class Stats : public wpi::Sendable, public CoreStats {
public:
	inline Stats(bool all_cores = false) : CoreStats(all_cores) {}

	virtual void InitSendable(wpi::SendableBuilder& b) override {
		b.AddFloatProperty("Core temp (C)", [this](){ return this->temp(); }, nullptr);
		b.AddFloatProperty("Core freq (Mhz)", [this](){ return this->freq_vcmd() / 1e6f; }, nullptr);
		b.AddFloatProperty("Core volts (V)", [this](){ return this->volts(); }, nullptr);
		b.AddIntegerProperty("Throttle State", [this](){ return this->throttlebits(); }, nullptr);
		b.AddFloatProperty("Utilization (%)", [this](){ return this->fromLast(); }, nullptr);
		b.AddFloatProperty("Main UpTime", [this](){ return this->mtime; }, nullptr);
	}

	float mtime{0.f};

};

struct CThread {
	CThread() = default;
	CThread(CThread&& t) :
		camera_matrix(std::move(t.camera_matrix)),
		dist_matrix(std::move(t.dist_matrix)),
		vmode(std::move(t.vmode)),
		vid(t.vid),
		camera_h(t.camera_h),
		fout_h(t.fout_h),
		fin_h(t.fin_h),
		view_h(t.view_h),
		link_state(t.link_state.load()),
		vpstate(t.vpstate.load()),
		vproc(std::move(t.vproc)),
		vpipe(std::move(t.vpipe)) {}

	cv::Mat1f
		camera_matrix{ Constant::DEFAULT_CAM_MATX },
		dist_matrix{ Constant::DEFAULT_CAM_MATX };
	cs::VideoMode vmode;

	int vid{-1};
	CS_Source camera_h, fout_h;
	CS_Sink fin_h, view_h;

	std::atomic<bool> link_state{true};
	std::atomic<int> vpstate{0};
	std::thread vproc;
	struct {
		cv::Mat frame, aframe, dframe;
		std::array<float, 11> timings;
		nt::FloatArrayEntry nt_timings;
	} vpipe;

};
void _update(CThread&);	// these are like instance methods
void _worker(CThread&);
void _shutdown(CThread&);






namespace util {

	template<typename f>
	cv::Point3_<f> wpiToCv(const cv::Point3_<f>& p) {
		return cv::Point3_<f>( -p.y, -p.z, p.x );
	}

	template<typename u = units::inch_t>
	frc::Translation3d cvToWpi_T(const cv::Mat1f& tvec) {
		return frc::Translation3d(
			u{ +tvec.at<float>(2, 0) },
			u{ -tvec.at<float>(0, 0) },
			u{ -tvec.at<float>(1, 0) }
		);
	}
	frc::Rotation3d cvToWpi_R(const cv::Mat1f& rvec) {
		frc::Vectord<3> rv{
			+rvec.at<float>(2, 0),
			-rvec.at<float>(0, 0),
			-rvec.at<float>(1, 0)
		};
		return frc::Rotation3d( rv, units::radian_t{ rv.norm() } );
	}

	template<typename u = units::inch_t>
	frc::Pose3d cvToWpi_P(const cv::Mat1f& tvec, const cv::Mat1f& rvec) {
		return frc::Pose3d{
			cvToWpi_T<u>(tvec),
			cvToWpi_R(rvec)
		};
	}
	template<typename u = units::inch_t>
	frc::Transform3d cvToWpi(const cv::Mat1f& tvec, const cv::Mat1f& rvec) {
		return frc::Transform3d{
			cvToWpi_T<u>(tvec),
			cvToWpi_R(rvec)
		};
	}

	template<typename u = units::inch_t>
	frc::Pose3d cvToWpiInv_P(const cv::Mat1f& tvec, const cv::Mat1f& rvec) {
		cv::Mat1f R, t;
		cv::Rodrigues(rvec, R);
		R = R.t();
		t = -R * tvec;
		return cvToWpi_P<u>(t, rvec);
	}
	template<typename u = units::inch_t>
	frc::Transform3d cvToWpiInv(const cv::Mat1f& tvec, const cv::Mat1f& rvec) {
		cv::Mat1f R, t;
		cv::Rodrigues(rvec, R);
		R = R.t();
		t = -R * tvec;
		return cvToWpi<u>(t, rvec);
	}

	// frc::Pose3d genPose(const std::vector<cv::Point3f>& corners, const frc::Vectord<3>& basedir = frc::Vectord<3>{0, 1, 0}) {
	// 	cv::Point3d center = ::findCenter3D<double>(corners);
	// 	frc::Vectord<3>
	// 		a{corners[4].x - corners[1].x, corners[4].y - corners[1].y, corners[4].z - corners[1].z},	// tl to bl
	// 		b{corners[2].x - corners[1].x, corners[2].y - corners[1].y, corners[2].z - corners[1].z},	// tl to tr
	// 		n = a.cross(b);		// normal vector for clockwise ordered corners
	// 	return frc::Pose3d(
	// 		*reinterpret_cast<frc::Translation3d*>(&center),
	// 		frc::Rotation3d(basedir, n)
	// 	);
	// }
	// std::vector<frc::Pose3d> genPoses(const std::vector<std::vector<cv::Point3f>>& tags, const frc::Vectord<3>& basedir = frc::Vectord<3>{0, 1, 0}) {
	// 	std::vector<frc::Pose3d> poses;
	// 	poses.reserve(tags.size());
	// 	for(const auto& tag : tags) {
	// 		poses.emplace_back(genPose(tag, basedir));
	// 	}
	// 	return poses;
	// }

};






/* Global Storage */
static struct {
	system_clock::time_point start_time;
	struct {
		std::atomic<bool>
			program_enable{true},
			view_updated{false},
			vrbo_updated{false},
			exposure_updated{false},
			dscale_updated{false},
			vretro_updated{false},
			perfm_updated{false}
		;
	} state;
	Stats stats{};

	std::shared_ptr<nt::NetworkTable> base_ntable;
	struct {
		nt::IntegerEntry
			view_id,		// active camera id
			ovl_verbosity,	// overlay verbosity - 0 for off, 1+ for increasing verbosity outputs
			april_mode,		// apriltag detection mode - -2 for off, -1 for active camera, 0+ for specific camera
			retro_mode,		// retrorefl tape detection mode - -2 for off, -1 for active camera, 0+ for specific camera
			exposure,		// exposure to apply to all cameras - default 40
			downscale,		// outputs are downscaled by this factor
			performance_override	// prefer vision pipelines or streams
		;
		nt::DoubleArrayEntry
			i_poses,
			c_poses,
			nodes,
			nodes_3d;
		nt::FloatArrayEntry
			pose_rmse,
			pose_distances,
			april_timings,
			retro_timings;
	} nt;

	int next_stream_port = 1181;
	std::vector<CThread> cthreads;
	cv::Mat disconnect_frame;
	CS_Source discon_frame_h;
	CS_Sink stream_h;


	const cv::Ptr<cv::aruco::DetectorParameters> aprilp_params{ cv::aruco::DetectorParameters::create() };
	const cv::Ptr<cv::aruco::Board> aprilp_field{ ::FIELD_2024 };
	const std::array<frc::Pose3d, 16>& aprilp_field_poses{ ::TAG_POSES_2024 };
	struct {
		std::thread april_worker, retro_worker;
		std::atomic<int> april_link{0}, retro_link{0};
		std::array<float, 3> april_profiling;
		std::array<float, 5> retro_profiling;
		struct {
			std::vector<std::vector<cv::Point2f>> tag_corners;
			std::vector<int32_t> tag_ids;
			std::vector<cv::Point2f> img_points;
			std::vector<cv::Point3f> obj_points;
			std::vector<cv::Mat1f> tvecs, rvecs;
			std::vector<frc::Pose3d> i_estimations, c_estimations;
			std::vector<float> eerrors, distances;
		} apbuff;
		struct {
			cv::Mat decimate, binary;
			std::vector<std::vector<cv::Point2i>> contours;
			std::vector<cv::Point2d> centers;
			std::vector<cv::Rect2i> bboxes;
#if ESTIMATE_TAPE_DISTANCES > 0
			cv::Mat1f tvec, rvec;
			std::vector<frc::Translation3d> locations;
#endif
#if VPIPE_CONFIG_NT_OPTIONS > 0
			nt::IntegerEntry
				nt_wst_alpha,
				nt_wst_beta,
				nt_wst_thresh,
				nt_wst_src_exposure;
			nt::FloatEntry
				nt_tape_lower_wh,
				nt_tape_upper_wh,
				nt_tape_fill_thresh,
				nt_tape_pix_thresh;
#endif
		} rtbuff;
	} vpp;	// 'Vision Processing Pipeline'

} _global;

void _april_worker_inst(CThread&, const cv::Mat* = nullptr);
void _retro_worker_inst(CThread&, const cv::Mat* = nullptr);
void _april_worker(CThread&, const cv::Mat* = nullptr);
void _retro_worker(CThread&, const cv::Mat* = nullptr);

bool init(const char* f = FRC_CONFIG);







int main(int argc, char** argv) {

	int status = 0;
	// setup
	{
		_global.start_time = system_clock::now();
		init();

		struct sigaction _action;
		sigemptyset(&(_action.sa_mask));
		_action.sa_flags = SA_SIGINFO;
		_action.sa_sigaction = [&_global](int, siginfo_t*, void*){
			_global.state.program_enable = false;
		};
		sigaction((unsigned int)SigHandle::Sigs::INT, &_action, nullptr);
		sigaction((unsigned int)SigHandle::Sigs::QUIT, &_action, nullptr);
		sigaction((unsigned int)SigHandle::Sigs::ILL, &_action, nullptr);
		sigaction((unsigned int)SigHandle::Sigs::ABRT, &_action, nullptr);
		sigaction((unsigned int)SigHandle::Sigs::SEGV, &_action, nullptr);
	}

#if DEBUG > 0
	int i = 0;
	for(CThread& t : _global.cthreads) {
		fmt::print(
			"Camera Instance {}:\n\tVID: {}\n\tSource handle: {}\n"
			"\tCV in handle: {}\n\tCV out handle: {}\n\tVMODE:\n"
			"\t\tFrame Width: {}\n\t\tFrame Height: {}\n"
			"\t\tFPS: {}\n\t\tFormat: {}\n",
			i++, t.vid, t.camera_h, t.fin_h, t.fout_h,
			t.vmode.width, t.vmode.height, t.vmode.fps, t.vmode.pixelFormat
		);
	}
#endif

	// mainloop
	for(;_global.state.program_enable;) {

		high_resolution_clock::time_point b = high_resolution_clock::now();

		_global.state.view_updated = (_global.nt.view_id.ReadQueue().size() > 0U);
		_global.state.vrbo_updated = (_global.nt.ovl_verbosity.ReadQueue().size() > 0U);
		_global.state.exposure_updated = (_global.nt.exposure.ReadQueue().size() > 0U);
		_global.state.dscale_updated = (_global.nt.downscale.ReadQueue().size() > 0U);
		_global.state.vretro_updated = (_global.nt.retro_mode.ReadQueue().size() > 0U);
		_global.state.perfm_updated = (_global.nt.performance_override.ReadQueue().size() > 0U);

#if DEBUG > 1
		if(_global.state.view_updated) { std::cout << "MAINLOOP: View idx updated." << std::endl; }
		if(_global.state.vrbo_updated) { std::cout << "MAINLOOP: Verbosity lvl updated." << std::endl; }
		if(_global.state.exposure_updated) { std::cout << "MAINLOOP: Exposure updated." << std::endl; }
		if(_global.state.dscale_updated) { std::cout << "MAINLOOP: Downscale updated." << std::endl; }
		if(_global.state.perfm_updated) { std::cout << "MAINLOOP: Performance mode updated." << std::endl; }
#endif

		for(CThread& t : _global.cthreads) {
			_update(t);
		}
		frc::SmartDashboard::UpdateValues();

		_global.stats.mtime = duration<float>(high_resolution_clock::now() - b).count();

		std::this_thread::sleep_until(b + 100ms);
	}

	// shutdown
	{
		std::cout << "\nShutting down threads..." << std::endl;
		for(CThread& t : _global.cthreads) { _shutdown(t); }
		if(_global.vpp.april_worker.joinable()) { _global.vpp.april_worker.join(); }
		if(_global.vpp.retro_worker.joinable()) { _global.vpp.retro_worker.join(); }
		cs::ReleaseSink(_global.stream_h, &status);
		cs::ReleaseSource(_global.discon_frame_h, &status);
		cs::Shutdown();
		std::cout << "Shutdown complete. Exitting..." << std::endl;
	}

	std::cout << "Runtime: " <<
		duration<double>(system_clock::now() -
			_global.start_time).count() << 's' << std::endl;

	return EXIT_SUCCESS;

}





bool loadJson(wpi::json& j, const char* file) {
	std::error_code ec;
	wpi::raw_fd_istream is(file, ec);
	if (ec) {
		wpi::errs() << "Could not open '" << file << "': " << ec.message() << newline;
		return false;
	}
	try { j = wpi::json::parse(is); }
	catch (const wpi::json::parse_error& e) {
		wpi::errs() << "Failed to parse JSON for " << file << /*": byte " << (int)e.byte <<*/ ": " << e.what() << newline;
		return false;
	}
	if (!j.is_object()) {
		wpi::errs() << "JSON error in " << file << ": not a JSON object\n";
		return false;
	}
	wpi::errs().flush();
	return true;
}
bool init(const char* fname) {

	high_resolution_clock::time_point start = high_resolution_clock::now();
	int status = 0;

	wpi::json j;
	loadJson(j, fname);
	if(j.count("ntmode") > 0) {
		try{
			std::string mode = j.at("ntmode").get<std::string>();
			if(wpi::equals_lower(mode, "server")) {
				nt::NetworkTableInstance::GetDefault().StartServer();
				std::cout << "Setup NT as SERVER" << std::endl;
			} else {
				int tnum = j.at("team").get<int>();
				std::array<std::pair<std::string_view, unsigned int>, 6> servers{{
					{SIM_ADDR, 0U}, {"172.22.11.2", 0U},
					{fmt::format("10.{}.{}.2", (tnum / 100), (tnum % 100)), 0U},
					{fmt::format("roboRIO-{}-FRC.local", tnum), 0U},
					{fmt::format("roboRIO-{}-FRC.lan", tnum), 0U},
					{fmt::format("roboRIO-{}-FRC.frc-field.local", tnum), 0U},
				}};
				nt::NetworkTableInstance::GetDefault().StartClient4(NT_IDENTITY);
				nt::NetworkTableInstance::GetDefault().SetServer(servers);
				std::cout << "Setup NT as CLIENT" << std::endl;
			}
		} catch(const wpi::json::exception& e) {
			std::cout << "NT config error in '" << fname << "': could not read 'ntmode': " << e.what() << std::endl;
			goto nt_fallback;
		}
	} else {
		nt_fallback:
#if NT_DEFAULT > 0
		nt::NetworkTableInstance::GetDefault().StartServer();
		std::cout << "Config file fallback: Setup NT as SERVER" << std::endl;
#else
		nt::NetworkTableInstance::GetDefault().StartClient4(NT_IDENTITY);
		nt::NetworkTableInstance::GetDefault().SetServer({{SIM_ADDR, "172.22.11.2"}});
		std::cout << "Config file fallback: Setup NT as CLIENT" << std::endl;
#endif
	}

	using namespace Constant;

	_global.base_ntable = nt::NetworkTableInstance::GetDefault().GetTable("Vision");
	_global.nt.view_id = _global.base_ntable->GetIntegerTopic("Active Camera Thread").GetEntry(0, NT_OPTIONS);
	_global.nt.ovl_verbosity = _global.base_ntable->GetIntegerTopic("Overlay Verbosity").GetEntry(0, NT_OPTIONS);
	_global.nt.exposure = _global.base_ntable->GetIntegerTopic("Camera Exposure").GetEntry(0, NT_OPTIONS);
	_global.nt.downscale = _global.base_ntable->GetIntegerTopic("Output Downscale").GetEntry(0, NT_OPTIONS);
	_global.nt.april_mode = _global.base_ntable->GetIntegerTopic("AprilTag Mode").GetEntry(-2, NT_OPTIONS);
	_global.nt.retro_mode = _global.base_ntable->GetIntegerTopic("RetroRefl Mode").GetEntry(-2, NT_OPTIONS);
	_global.nt.performance_override = _global.base_ntable->GetIntegerTopic("Performance Mode").GetEntry(0, NT_OPTIONS);
	_global.nt.i_poses = _global.base_ntable->GetDoubleArrayTopic("Pose Estimations/Individual").GetEntry({}, NT_DATAFLOW_OPTIONS);
	_global.nt.c_poses = _global.base_ntable->GetDoubleArrayTopic("Pose Estimations/Combined").GetEntry({}, NT_DATAFLOW_OPTIONS);
	_global.nt.pose_rmse = _global.base_ntable->GetFloatArrayTopic("Pose Estimations/RMSE").GetEntry({}, NT_DATAFLOW_OPTIONS);
	_global.nt.pose_distances = _global.base_ntable->GetFloatArrayTopic("Pose Estimations/Tag Distances").GetEntry({}, NT_DATAFLOW_OPTIONS);
	_global.nt.nodes = _global.base_ntable->GetDoubleArrayTopic("RetroReflective Detections/Centers").GetEntry({}, NT_DATAFLOW_OPTIONS);
#if ESTIMATE_TAPE_DISTANCES > 0
	_global.nt.nodes_3d = _global.base_ntable->GetDoubleArrayTopic("RetroReflective Detections/Translations").GetEntry({}, NT_DATAFLOW_OPTIONS);
#endif
	std::shared_ptr<nt::NetworkTable> thread_stats_nt = _global.base_ntable->GetSubTable("Threads");
	_global.nt.april_timings = thread_stats_nt->GetFloatArrayTopic("AprilTag Pipeline").GetEntry({}, NT_OPTIONS);
	_global.nt.retro_timings = thread_stats_nt->GetFloatArrayTopic("RetroRef Pipeline").GetEntry({}, NT_OPTIONS);

	_global.disconnect_frame = cv::Mat::zeros({DEFAULT_VMODE.width, DEFAULT_VMODE.height}, CV_8UC3);
	cv::putText(
		_global.disconnect_frame, "Camera is Unavailable :(",
		cv::Point(DEFAULT_VMODE.width / 8, DEFAULT_VMODE.height / 2),
		cv::FONT_HERSHEY_SIMPLEX, 1.2, {0, 0, 255}, 4, cv::LINE_AA
	);
	_global.discon_frame_h = cs::CreateCvSource("Disconnected Frame Source", DEFAULT_VMODE, &status);
	_global.stream_h = cs::CreateMjpegServer("Viewport Stream", "", _global.next_stream_port++, &status);
#if ENABLE_CAMERASERVER > 0
	frc::CameraServer::AddServer(VideoSinkImpl(_global.stream_h));
#endif
#if DEBUG > 0
	std::cout << fmt::format("Created main stream with port {}.", _global.next_stream_port - 1) << std::endl;
#endif

	std::vector<cs::UsbCameraInfo> connections = cs::EnumerateUsbCameras(&status);

	int vid_additions = CamID::NUM_CAMERAS;
	try {
		for(const wpi::json& camera : j.at("cameras")) {
			CThread& cthr = _global.cthreads.emplace_back();
			std::string name = camera.at("name").get<std::string>();
			std::string path = camera.at("path").get<std::string>();

			cthr.camera_h = cs::CreateUsbCameraPath(fmt::format("{}_src", name), path, &status);
			cs::SetSourceConfigJson(cthr.camera_h, camera, &status);	// set to DEFAULT_VMODE if invalid
			cs::SetSourceConnectionStrategy(cthr.camera_h, CS_CONNECTION_KEEP_OPEN, &status);
			cthr.vmode = cs::GetSourceVideoMode(cthr.camera_h, &status);
#if PATCH_LIFECAM_V4L_PROP_IDS > 0
			cs::SetProperty(cs::GetSourceProperty(cthr.camera_h, "auto_exposure", &status), 1, &status);
			cs::SetProperty(cs::GetSourceProperty(cthr.camera_h, "exposure_time_absolute", &status), DEFAULT_EXPOSURE, &status);
#else
			cs::SetCameraExposureManual(cthr.camera_h, DEFAULT_EXPOSURE, &status);
#endif
			cs::SetCameraWhiteBalanceAuto(cthr.camera_h, &status);
			cs::SetCameraBrightness(cthr.camera_h, DEFAULT_BRIGHTNESS, &status);

			cthr.vpipe.nt_timings = thread_stats_nt->GetFloatArrayTopic(fmt::format("{} camera", name)).GetEntry({}, NT_OPTIONS);
			cthr.vpipe.nt_timings.Set({});

			cthr.fin_h = cs::CreateCvSink(
				fmt::format("{}_cv_in", name), &status);
			cthr.fout_h = cs::CreateCvSource(
				fmt::format("{}_cv_out", name), cthr.vmode, &status);
			cthr.view_h = cs::CreateMjpegServer(
				fmt::format("{}_view_stream", name), "", _global.next_stream_port++, &status);
#if ENABLE_CAMERASERVER > 0
			frc::CameraServer::AddServer(VideoSinkImpl(cthr.view_h));
#endif
#if DEBUG > 0
			std::cout << fmt::format("Created {} camera view stream with port {}.", name, _global.next_stream_port - 1) << std::endl;
#endif
			// cs::SetSinkSource(cthr.view_h, cthr.camera_h, &status);
			cs::SetSinkSource(cthr.view_h, cthr.fout_h, &status);		// there is some sort of bug where these streams don't switch, so start with the one more likely to be used

			for(size_t t = 0; t < CAMERA_TAGS.size(); t++) {
				if(wpi::equals_lower(name, CAMERA_TAGS[t])) {
					cthr.vid = t;
					break;
				}
			}
			if(cthr.vid < 0) { cthr.vid = vid_additions++; }

			const decltype(STATIC_CALIBRATIONS)::Cal_T* cal = nullptr;
			if(cal = findCalib(name, {cthr.vmode.width, cthr.vmode.height}, STATIC_CALIBRATIONS)) {
				std::cout << fmt::format("Found calibration for camera '{}' by name.", name) << std::endl;
				cthr.camera_matrix = cal->at(0);
				cthr.dist_matrix = cal->at(1);
#if DEBUG > 0
				std::cout << "CMatx: " << cthr.camera_matrix << std::endl;
				std::cout << "DCoefs: " << cthr.dist_matrix << std::endl;
#endif
			}

			for(cs::UsbCameraInfo& info : connections) {
				if(wpi::equals_lower(info.path, path)) {
					info.dev = -1;
#if DEBUG > 0
					std::cout << fmt::format("Camera path '{}' classified as type '{}'.", path, info.name) << std::endl;
#endif
					if(!cal && (cal = findCalib(info.name, {cthr.vmode.width, cthr.vmode.height}, STATIC_CALIBRATIONS))) {	// maybe search by path too?
						std::cout << fmt::format("Found calibration for camera '{}' by type.", name) << std::endl;
						cthr.camera_matrix = cal->at(0);
						cthr.dist_matrix = cal->at(1);
#if DEBUG > 0
						std::cout << "CMatx: " << cthr.camera_matrix << std::endl;
						std::cout << "DCoefs: " << cthr.dist_matrix << std::endl;
#endif
					}
					break;
				}
			}

			if(cal == nullptr) {
				std::cout << fmt::format("Failed to find calibration for camera '{}'.", name) << std::endl;
			}

		}
	} catch(const wpi::json::exception& e) {
		std::cout << "Config file error in '" << fname << "': Could not read camera configuration: " << e.what() << std::endl;
	}
	for(size_t i = 0; i < connections.size(); i++) {
		if(connections[i].dev >= 0 && strncmp(connections[i].name.c_str(), "bcm", 3) != 0) {
			CThread& cthr = _global.cthreads.emplace_back();
			cthr.vid = vid_additions++;
			cthr.camera_h = cs::CreateUsbCameraPath(
				fmt::format("Camera{}_src", cthr.vid), connections[i].path, &status);
			cs::SetSourceConnectionStrategy(cthr.camera_h, CS_CONNECTION_KEEP_OPEN, &status);
			cs::SetSourceVideoMode(cthr.camera_h, (cthr.vmode = DEFAULT_VMODE), &status);
#if PATCH_LIFECAM_V4L_PROP_IDS > 0
			cs::SetProperty(cs::GetSourceProperty(cthr.camera_h, "auto_exposure", &status), 1, &status);
			cs::SetProperty(cs::GetSourceProperty(cthr.camera_h, "exposure_time_absolute", &status), DEFAULT_EXPOSURE, &status);
#else
			cs::SetCameraExposureManual(cthr.camera_h, DEFAULT_EXPOSURE, &status);
#endif
			cs::SetCameraWhiteBalanceAuto(cthr.camera_h, &status);
			cs::SetCameraBrightness(cthr.camera_h, DEFAULT_BRIGHTNESS, &status);

			cthr.vpipe.nt_timings = thread_stats_nt->GetFloatArrayTopic(fmt::format("Camera{}", cthr.vid)).GetEntry({}, NT_OPTIONS);
			cthr.vpipe.nt_timings.Set({});

			cthr.fin_h = cs::CreateCvSink(
				fmt::format("Camera{}_cv_in", cthr.vid), &status);
			cthr.fout_h = cs::CreateCvSource(
				fmt::format("Camera{}_cv_out", cthr.vid), cthr.vmode, &status);
			cthr.view_h = cs::CreateMjpegServer(
				fmt::format("Camera{}_view_stream", cthr.vid), "", _global.next_stream_port++, &status);
#if ENABLE_CAMERASERVER > 0
			frc::CameraServer::AddServer(VideoSinkImpl(cthr.view_h));
#endif
#if DEBUG > 0
			std::cout << fmt::format("Created Camera{} view stream with port {}.", cthr.vid, _global.next_stream_port - 1) << std::endl;
#endif
			// cs::SetSinkSource(cthr.view_h, cthr.camera_h, &status);
			cs::SetSinkSource(cthr.view_h, cthr.fout_h, &status);

			const decltype(STATIC_CALIBRATIONS)::Cal_T* cal = nullptr;
			if(cal = findCalib(connections[i].name, {cthr.vmode.width, cthr.vmode.height}, STATIC_CALIBRATIONS)) {	// maybe search by path too?
				std::cout << fmt::format("Found calibration for 'Camera{}' by type.", cthr.vid) << std::endl;
				cthr.camera_matrix = cal->at(0);
				cthr.dist_matrix = cal->at(1);
#if DEBUG > 0
				std::cout << "CMatx: " << cthr.camera_matrix << std::endl;
				std::cout << "DCoefs: " << cthr.dist_matrix << std::endl;
#endif
			}
		}
	}

	std::cout << fmt::format("Cameras Available: {}", _global.cthreads.size()) << std::endl;

	_global.nt.view_id.Set(_global.cthreads.size() > 0 ? _global.cthreads[0].vid : -1);
	_global.nt.ovl_verbosity.Set(1);
	_global.nt.exposure.Set(DEFAULT_EXPOSURE);
	_global.nt.downscale.Set(DEFAULT_DOWNSCALE);
	_global.nt.april_mode.Set(-1);
	_global.nt.retro_mode.Set(-2);
	_global.nt.performance_override.Set(0);

#if VPIPE_CONFIG_NT_OPTIONS > 0
	std::shared_ptr<nt::NetworkTable> vpipe_tuning_nt = _global.base_ntable->GetSubTable("Tuning");
	_global.vpp.rtbuff.nt_wst_alpha = vpipe_tuning_nt->GetIntegerTopic("Retro WST Alpha").GetEntry(VRetro::WST_ALPHA, NT_OPTIONS);
	_global.vpp.rtbuff.nt_wst_beta = vpipe_tuning_nt->GetIntegerTopic("Retro WST Beta").GetEntry(VRetro::WST_BETA, NT_OPTIONS);
	_global.vpp.rtbuff.nt_wst_thresh = vpipe_tuning_nt->GetIntegerTopic("Retro WST Thresh").GetEntry(VRetro::WST_THRESH, NT_OPTIONS);
	_global.vpp.rtbuff.nt_wst_src_exposure = vpipe_tuning_nt->GetIntegerTopic("RetroRef Exposure").GetEntry(VRetro::SRC_EXPOSURE, NT_OPTIONS);
	_global.vpp.rtbuff.nt_tape_lower_wh = vpipe_tuning_nt->GetFloatTopic("Retro Min Aspect").GetEntry(VRetro::TAPE_LOWER_WH_RATIO, NT_OPTIONS);
	_global.vpp.rtbuff.nt_tape_upper_wh = vpipe_tuning_nt->GetFloatTopic("Retro Max Aspect").GetEntry(VRetro::TAPE_UPPER_WH_RATIO, NT_OPTIONS);
	_global.vpp.rtbuff.nt_tape_fill_thresh = vpipe_tuning_nt->GetFloatTopic("Retro Fill Percent").GetEntry(VRetro::TAPE_RECT_FILL_THRESH, NT_OPTIONS);
	_global.vpp.rtbuff.nt_tape_pix_thresh = vpipe_tuning_nt->GetFloatTopic("Retro Pixel Thresh").GetEntry(VRetro::TAPE_PIXEL_COUNT_THRESH, NT_OPTIONS);
	_global.vpp.rtbuff.nt_wst_alpha.Set(VRetro::WST_ALPHA);
	_global.vpp.rtbuff.nt_wst_beta.Set(VRetro::WST_BETA);
	_global.vpp.rtbuff.nt_wst_thresh.Set(VRetro::WST_THRESH);
	_global.vpp.rtbuff.nt_wst_src_exposure.Set(VRetro::SRC_EXPOSURE);
	_global.vpp.rtbuff.nt_tape_lower_wh.Set(VRetro::TAPE_LOWER_WH_RATIO);
	_global.vpp.rtbuff.nt_tape_upper_wh.Set(VRetro::TAPE_UPPER_WH_RATIO);
	_global.vpp.rtbuff.nt_tape_fill_thresh.Set(VRetro::TAPE_RECT_FILL_THRESH);
	_global.vpp.rtbuff.nt_tape_pix_thresh.Set(VRetro::TAPE_PIXEL_COUNT_THRESH);
#endif

	frc::SmartDashboard::init();
	frc::SmartDashboard::PutData("Vision/Stats", &_global.stats);

	// _global.aprilp_params->useAruco3Detection = false;
	// _global.aprilp_params->aprilTagQuadDecimate = 2.0;

	std::cout << fmt::format("Initialization completed in {}s.",
		duration<double>(high_resolution_clock::now() - start).count()) << std::endl;
	
	return true;
}










void _update(CThread& ctx) {
	using namespace Constant;

	int status = 0;
	const int apmode = _global.nt.april_mode.Get();
	const int rflmode = _global.nt.retro_mode.Get();
	const int perfm = _global.nt.performance_override.Get();
	const bool downscale = (_global.nt.downscale.Get() > 1) && (perfm < PerfMode::VPP_ONLY);
	const bool overlay = (_global.nt.ovl_verbosity.Get() > 0) && (perfm <= PerfMode::NONE || perfm == PerfMode::SINGLESTREAM);
	const bool active = ctx.vid == _global.nt.view_id.Get();
	const bool multistream = (perfm <= PerfMode::RAW_MULTISTREAM);	// and maybe add a constexpr override for completely disabling?
	const bool mainstream = active && (perfm < PerfMode::VPP_ONLY);
	const bool streaming = multistream || mainstream;
	const bool connected = cs::IsSourceConnected(ctx.camera_h, &status);
	ctx.vpstate = (	// first bit is apmode, second is reflmode
		((int)( (perfm > PerfMode::STREAM_ONLY) && (perfm != PerfMode::RETRO_ONLY) && (((apmode == -1)	&& active) || (apmode == ctx.vid)) ) << 0) |
		((int)( (perfm > PerfMode::STREAM_ONLY) && (perfm != PerfMode::APRIL_ONLY) && (((rflmode == -1)	&& active) || (rflmode == ctx.vid)) ) << 1)
	);
	const bool enable_thread = connected && (((overlay || downscale) && streaming) || ctx.vpstate);
																	// ^^ this is possibly a redundant comparison but still include it for sanity
	if(_global.state.view_updated || _global.state.vrbo_updated || _global.state.dscale_updated || _global.state.perfm_updated) {
		// cs::SetSinkEnabled(ctx.view_h, multistream, &status);	// this does not actually do anything --> instead apply to CvSink's
		// if(active) {
		// 	cs::SetSinkEnabled(_global.stream_h, mainstream, &status);
		// }
		if(connected) {
			if(overlay || downscale) {
				if(multistream) cs::SetSinkSource(ctx.view_h, ctx.fout_h, &status);
				if(mainstream) cs::SetSinkSource(_global.stream_h, ctx.fout_h, &status);
			} else {
				if(multistream) cs::SetSinkSource(ctx.view_h, ctx.camera_h, &status);
					else cs::SetSinkSource(ctx.view_h, ctx.fout_h, &status);				// or set to a dummy source to ensure no streaming
				if(mainstream) cs::SetSinkSource(_global.stream_h, ctx.camera_h, &status);
					else if(active) cs::SetSinkSource(_global.stream_h, ctx.fout_h, &status);	// ^^^ in theory, if !(overlay || downscale), then the thread should never output to ctx.fout_h
			}
		} else {
			if(multistream) cs::SetSinkSource(ctx.view_h, _global.discon_frame_h, &status);
			if(mainstream) cs::SetSinkSource(_global.stream_h, _global.discon_frame_h, &status);
		}
	}

	const bool possible_exp_update = _global.state.vretro_updated || _global.state.perfm_updated || _global.state.view_updated;

#if PATCH_LIFECAM_V4L_PROP_IDS <= 0
	if(ctx.vpstate & 0b10) {	// retrorefl pipe is operating on the thread
		if(possible_exp_update
#if VPIPE_CONFIG_NT_OPTIONS > 0
			|| _global.vpp.rtbuff.nt_wst_src_exposure.ReadQueue().size() > 0
#endif
		) {
#if VPIPE_CONFIG_NT_OPTIONS > 0
			const int exp = _global.vpp.rtbuff.nt_wst_src_exposure.Get();
			if(exp < 0) {
				cs::SetCameraExposureAuto(ctx.camera_h, &status);
			} else {
				cs::SetCameraExposureManual(ctx.camera_h, exp, &status);
			}
#else
			if constexpr(VRetro::SRC_EXPOSURE < 0) {
				cs::SetCameraExposureAuto(ctx.camera_h, &status);
			} else {
				cs::SetCameraExposureManual(ctx.camera_h, VRetro::SRC_EXPOSURE, &status);
			}
#endif
		}
	} else
#endif
	if(_global.state.exposure_updated || possible_exp_update) {
		const int exp = _global.nt.exposure.Get();
#if PATCH_LIFECAM_V4L_PROP_IDS > 0
		if(exp < 0) {
			cs::SetProperty(cs::GetSourceProperty(ctx.camera_h, "auto_exposure", &status), 3, &status);
		} else {
			cs::SetProperty(cs::GetSourceProperty(ctx.camera_h, "auto_exposure", &status), 1, &status);
			cs::SetProperty(cs::GetSourceProperty(ctx.camera_h, "exposure_time_absolute", &status), exp, &status);
		}
#else
		if(exp < 0) {
			cs::SetCameraExposureAuto(ctx.camera_h, &status);
		} else {
			cs::SetCameraExposureManual(ctx.camera_h, exp, &status);
		}
#endif
	}
	if(enable_thread) {
		if(!ctx.vproc.joinable()) {
			ctx.link_state = true;
			ctx.vproc = std::thread(_worker, std::ref(ctx));
		}
	} else if(ctx.vproc.joinable()) {
		ctx.link_state = false;
		ctx.vproc.join();
	}
	if(mainstream && !connected) {
		cs::PutSourceFrame(_global.discon_frame_h, _global.disconnect_frame, &status);
	}
}

using AnnotationBuffer =
	std::tuple<
		float*, float*,
		std::vector<std::vector<cv::Point2f>>,
		std::vector<int32_t>,
		std::vector<std::vector<cv::Point2i>>,
		std::vector<cv::Rect2i>,
		std::mutex
	>;
void _annotate_output(CThread& ctx, std::atomic<int>& link, int vb, int ds, AnnotationBuffer* anno) {

	link = 1;

	high_resolution_clock::time_point ta, tb, tp = high_resolution_clock::now();
	decltype(ctx.vpipe.timings)& timing = ctx.vpipe.timings;
	const cv::Size2i fsz = ctx.vpipe.frame.size() / (ds < 1 ? 1 : ds);
	int status{0};

	if(ctx.vpipe.dframe.size() != fsz) {
		ctx.vpipe.dframe = cv::Mat(fsz, CV_8UC3);
	}
	cv::resize(ctx.vpipe.frame, ctx.vpipe.dframe, fsz, 0, 0, cv::INTER_AREA);
																				ta = high_resolution_clock::now();
																				timing[5] = duration<float>(ta - tp).count() * 1000.f;
	if(vb > 0) {
		ctx.vpipe.aframe = cv::Mat::zeros(fsz, CV_8UC3);
		const double
			fntscale1 = fsz.height / 1108.0,
			fntscale2 = fsz.height / 1440.0;
		const int
			fnthz1 = fsz.height / 24,
			fnthz2 = fsz.height / 48;

		cv::putText(ctx.vpipe.aframe,
					fmt::format("{:.1f}", 1000.f / timing[0]),
					cv::Point(5, fnthz1), cv::FONT_HERSHEY_DUPLEX, fntscale1, {0, 255, 0}, 1, cv::LINE_AA);
		int vl = 2 * fnthz1;

		if(vb > 1) {
			if(ctx.vpstate && anno) {
				if(ctx.vpstate & 0b01) {
					if(std::get<3>(*anno).size() > 0 && std::get<6>(*anno).try_lock()) {
						if(ds > 1) {
							::rescale(std::get<2>(*anno), 1.0 / ds);
						}
						cv::aruco::drawDetectedMarkers(ctx.vpipe.aframe, std::get<2>(*anno), std::get<3>(*anno));
						std::get<6>(*anno).unlock();
					}
					float* apt = std::get<0>(*anno);
					if(apt) {
						cv::putText(ctx.vpipe.aframe, fmt::format("[AP] Detection time: {:.3f}ms", apt[0] * 1000.f),
									cv::Point(5, vl += fnthz2), cv::FONT_HERSHEY_DUPLEX, fntscale2, {0, 255, 0}, 1, cv::LINE_AA);
						cv::putText(ctx.vpipe.aframe, fmt::format("[AP] Estimation time: {:.3f}ms", apt[1] * 1000.f),
									cv::Point(5, vl += fnthz2), cv::FONT_HERSHEY_DUPLEX, fntscale2, {0, 255, 0}, 1, cv::LINE_AA);
						cv::putText(ctx.vpipe.aframe, fmt::format("[AP] Total thread time: {:.3f}ms", apt[2] * 1000.f),
									cv::Point(5, vl += fnthz2), cv::FONT_HERSHEY_DUPLEX, fntscale2, {0, 255, 0}, 1, cv::LINE_AA);
					}
				}
				if(ctx.vpstate & 0b10) {
					if(std::get<4>(*anno).size() > 0 && std::get<6>(*anno).try_lock()) {
						float rscale = Constant::VRetro::DECIMATE_FACTOR / (ds < 1 ? 1 : ds);
						if(rscale != 1.f) {
							::rescale(std::get<4>(*anno), rscale);
							::rescale(std::get<5>(*anno), rscale);
						}
						cv::drawContours(ctx.vpipe.aframe, std::get<4>(*anno), -1, {0, 255, 0});
						for(const cv::Rect& r : std::get<5>(*anno)) {
							cv::rectangle(ctx.vpipe.aframe, r, {0, 255, 0}, 1, cv::LINE_AA);
						}
						std::get<6>(*anno).unlock();
					}
					float* rtt = std::get<1>(*anno);
					if(rtt) {
						cv::putText(ctx.vpipe.aframe, fmt::format("[RT] Init & rescale: {:.3f}ms", rtt[0] * 1000.f),
									cv::Point(5, vl += fnthz2), cv::FONT_HERSHEY_DUPLEX, fntscale2, {0, 255, 0}, 1, cv::LINE_AA);
						cv::putText(ctx.vpipe.aframe, fmt::format("[RT] (NEON) WSTB time: {:.3f}ms", rtt[1] * 1000.f),
									cv::Point(5, vl += fnthz2), cv::FONT_HERSHEY_DUPLEX, fntscale2, {0, 255, 0}, 1, cv::LINE_AA);
						cv::putText(ctx.vpipe.aframe, fmt::format("[RT] Find Contours: {:.3f}ms", rtt[2] * 1000.f),
									cv::Point(5, vl += fnthz2), cv::FONT_HERSHEY_DUPLEX, fntscale2, {0, 255, 0}, 1, cv::LINE_AA);
						cv::putText(ctx.vpipe.aframe, fmt::format("[RT] Filtering: {:.3f}ms", rtt[3] * 1000.f),
									cv::Point(5, vl += fnthz2), cv::FONT_HERSHEY_DUPLEX, fntscale2, {0, 255, 0}, 1, cv::LINE_AA);
						cv::putText(ctx.vpipe.aframe, fmt::format("[RT] Total thread time: {:.3f}ms", rtt[4] * 1000.f),
									cv::Point(5, vl += fnthz2), cv::FONT_HERSHEY_DUPLEX, fntscale2, {0, 255, 0}, 1, cv::LINE_AA);
					}
				}
			}

			cv::putText(ctx.vpipe.aframe,
						fmt::format("CThread: [Pm:{:.1f}ms, APt:{:.1f}ms, RTt:{:.1f}ms, AOt:{:.1f}ms, AOi:{:.1f}ms, AOa:{:.1f}ms, AOc:{:.1f}ms, AOo:{:.1f}ms, AO:{:.1f}ms]",
							timing[1], timing[2], timing[3], timing[4], timing[5], timing[6], timing[7], timing[8], timing[9]),
						cv::Point(5, fsz.height - fnthz1), cv::FONT_HERSHEY_DUPLEX, fntscale2, {0, 255, 0}, 1, cv::LINE_AA);
		}
																				tb = high_resolution_clock::now();
																				timing[6] = duration<float>(tb - ta).count() * 1000.f;
		neon_bitwise_or(ctx.vpipe.aframe, ctx.vpipe.dframe, ctx.vpipe.dframe);
																				timing[7] = duration<float>(high_resolution_clock::now() - tb).count() * 1000.f;
	} else { timing[5] = timing[6] = 0.f; }
																				ta = high_resolution_clock::now();
	cs::PutSourceFrame(ctx.fout_h, ctx.vpipe.dframe, &status);
																				tb = high_resolution_clock::now();
																				timing[8] = duration<float>(tb - ta).count() * 1000.f;
																				timing[9] = duration<float>(tb - tp).count() * 1000.f;
																				timing[10] = timing[2] + timing[3] + timing[9];
	ctx.vpipe.nt_timings.Set(std::span<float>(timing.begin(), timing.end()));

	link = 2;

}
void _worker(CThread& ctx) {

	high_resolution_clock::time_point ta, tf = high_resolution_clock::now();
	int status{0}, verbosity, downscale, perfmode;
	std::atomic<int> output_link{0};
	std::thread output_worker;
	decltype(ctx.vpipe.timings)& timing = ctx.vpipe.timings;
	AnnotationBuffer* annobuff{nullptr};

	cs::SetSinkEnabled(ctx.fin_h, true, &status);
	cs::SetSinkSource(ctx.fin_h, ctx.camera_h, &status);

	for(;ctx.link_state && _global.state.program_enable;) {

		cs::GrabSinkFrame(ctx.fin_h, ctx.vpipe.frame, &status);
																				ta = high_resolution_clock::now();
																				timing[0] = duration<float>(ta - tf).count() * 1000.f;
																				tf = ta;
		perfmode = _global.nt.performance_override.Get();
		verbosity = (_global.nt.ovl_verbosity.Get()) * (perfmode <= Constant::PerfMode::NONE || perfmode == Constant::PerfMode::SINGLESTREAM);
		downscale = (_global.nt.downscale.Get()) * (perfmode < Constant::PerfMode::VPP_ONLY);

		if(ctx.vpstate & 0b01) {		// apriltag
																				ta = high_resolution_clock::now();
			if(_global.vpp.april_link > 1 && _global.vpp.april_worker.joinable()) {
				_global.vpp.april_worker.join();
				_global.vpp.april_link = 0;
			}
			if(_global.vpp.april_link == 0) {	// if the thread is stopped
				if(verbosity > 1) {
					if(!annobuff) { annobuff = new AnnotationBuffer; }
					if(!std::get<0>(*annobuff)) { std::get<0>(*annobuff) = new float[_global.vpp.april_profiling.size()]; }
					memcpy(std::get<0>(*annobuff), _global.vpp.april_profiling.data(), _global.vpp.april_profiling.size() * sizeof(float));
					if(std::get<6>(*annobuff).try_lock()) {
						if(_global.vpp.apbuff.tag_corners.size() > 0) {
							std::get<2>(*annobuff) = _global.vpp.apbuff.tag_corners;
							std::get<3>(*annobuff) = _global.vpp.apbuff.tag_ids;
						} else {
							std::get<2>(*annobuff).clear();
							std::get<3>(*annobuff).clear();
						}
						std::get<6>(*annobuff).unlock();
					}
				}
				_global.vpp.april_worker = std::thread(_april_worker, std::ref(ctx), &ctx.vpipe.frame);
			}
																				timing[2] = duration<float>(high_resolution_clock::now() - ta).count() * 1000.f;
		} else { timing[2] = 0.f; }
		if(ctx.vpstate & 0b10) {		// retroreflective
																				ta = high_resolution_clock::now();
			if(_global.vpp.retro_link > 1 && _global.vpp.retro_worker.joinable()) {
				_global.vpp.retro_worker.join();
				_global.vpp.retro_link = 0;
			}
			if(_global.vpp.retro_link == 0) {	// if the thread is stopped
				if(verbosity > 1) {
					if(!annobuff) { annobuff = new AnnotationBuffer; }
					if(!std::get<1>(*annobuff)) { std::get<1>(*annobuff) = new float[_global.vpp.retro_profiling.size()]; }
					memcpy(std::get<1>(*annobuff), _global.vpp.retro_profiling.data(), _global.vpp.retro_profiling.size() * sizeof(float));
					if(std::get<6>(*annobuff).try_lock()) {
						if(_global.vpp.rtbuff.contours.size() > 0) {
							std::get<4>(*annobuff) = _global.vpp.rtbuff.contours;
							std::get<5>(*annobuff) = _global.vpp.rtbuff.bboxes;
						} else {
							std::get<4>(*annobuff).clear();
							std::get<5>(*annobuff).clear();
						}
						std::get<6>(*annobuff).unlock();
					}
				}
				_global.vpp.retro_worker = std::thread(_retro_worker, std::ref(ctx), &ctx.vpipe.frame);
			}
																				timing[3] = duration<float>(high_resolution_clock::now() - ta).count() * 1000.f;
		} else { timing[3] = 0.f; }
																				ta = high_resolution_clock::now();
		if(verbosity || downscale > 0) {	// annotation and output --> only if not directly streaming
			if(output_link > 1 && output_worker.joinable()) {
				output_worker.join();
				output_link = 0;
			}
			if(output_link == 0) {
				output_worker = std::thread(_annotate_output, std::ref(ctx), std::ref(output_link),
											verbosity, downscale, annobuff);
			}
		} else {
			timing[5] = timing[6] = timing[7] = timing[8] = timing[9] = timing[10] = 0.f;
			ctx.vpipe.nt_timings.Set(std::span<float>(timing.begin(), timing.end()));
		}
																				timing[4] = duration<float>(high_resolution_clock::now() - ta).count() * 1000.f;
																				timing[1] = duration<float>(high_resolution_clock::now() - tf).count() * 1000.f;

	}

	cs::SetSinkEnabled(ctx.fin_h, false, &status);

	if(output_worker.joinable()) {
		output_worker.join();
	}
	if(annobuff) { delete annobuff; }

}
void _shutdown(CThread& ctx) {
	int status = 0;
	if(ctx.vproc.joinable()) {
		ctx.link_state = false;
		ctx.vproc.join();
	}
	cs::ReleaseSource(ctx.camera_h, &status);
	cs::ReleaseSource(ctx.fout_h, &status);
	cs::ReleaseSink(ctx.fin_h, &status);
}










void _april_worker_inst(CThread& target, const cv::Mat* f) {

	_global.vpp.april_link = 1;

	high_resolution_clock::time_point p, beg = high_resolution_clock::now();

	decltype(_global.vpp.apbuff)& _buff = _global.vpp.apbuff;
	_buff.tag_corners.clear();
	_buff.tag_ids.clear();

	p = high_resolution_clock::now();
	cv::aruco::detectMarkers(
		(f ? *f : target.vpipe.frame),
		_global.aprilp_field->dictionary,
		_buff.tag_corners, _buff.tag_ids,
		_global.aprilp_params
	);
	_global.vpp.april_profiling[0] = duration<float>(high_resolution_clock::now() - p).count();

	if(const size_t detections = _buff.tag_ids.size(); (detections > 0 && detections == _buff.tag_corners.size())) {
		_buff.i_estimations.clear();
		_buff.c_estimations.clear();
		_buff.eerrors.clear();
		_buff.distances.clear();

		p = high_resolution_clock::now();
		{
			if(detections == 1) {	// single tag
				int id = _buff.tag_ids[0];
				if(id > _global.aprilp_field->ids.size()) {}

				cv::solvePnPGeneric(
					::GENERIC_TAG_CORNERS, _buff.tag_corners[0],
					target.camera_matrix, target.dist_matrix,
					_buff.rvecs, _buff.tvecs, false, cv::SOLVEPNP_IPPE_SQUARE,
					cv::noArray(), cv::noArray(), _buff.eerrors
				);

				for(size_t i = 0; i < _buff.tvecs.size(); i++) {
					frc::Transform3d c2tag = util::cvToWpi(_buff.tvecs[i], _buff.rvecs[i]);
					_buff.c_estimations.push_back(_global.aprilp_field_poses[id - 1].TransformBy(c2tag.Inverse()));
					_buff.distances.push_back(c2tag.Translation().Norm().value());
				}
				_buff.i_estimations = _buff.c_estimations;		// make a constexpr param for enabling/disabling this
			} else {				// 'megatag'
				using namespace cv;
#define MEGATAG_SOLVE_METHOD SOLVEPNP_SQPNP
#define SOLVE_COMPONENT_POSES 0

#if MEGATAG_SOLVE_METHOD > -1
				_buff.obj_points.clear();
				_buff.img_points.clear();
				_buff.obj_points.reserve(detections * 4U);
				_buff.obj_points.reserve(detections * 4U);
#endif
				// std::vector<float> rmse;

				for(size_t i = 0; i < detections; i++) {

					int id = _buff.tag_ids[i];
					if(id > _global.aprilp_field->ids.size()) { continue; }

					std::vector<cv::Point2f>& itag_corners = _buff.tag_corners[i];
					size_t iid;
					for(iid = 0; iid < _global.aprilp_field->ids.size(); iid++) {
						if(id == _global.aprilp_field->ids[iid]) {
							_buff.tag_ids[i] = iid;	// replace with array index for tag id (normally is the same)
#if MEGATAG_SOLVE_METHOD > -1
							_buff.img_points.insert(_buff.img_points.end(), itag_corners.begin(), itag_corners.end());
							for(int p = 0; p < 4; p++) {
								_buff.obj_points.push_back(util::wpiToCv(_global.aprilp_field->objPoints[iid][p]));
							}
#endif
							break;
						}
					}

#if SOLVE_COMPONENT_POSES > 0
					cv::solvePnPGeneric(
						::GENERIC_TAG_CORNERS, itag_corners,
						target.camera_matrix, target.dist_matrix,
						_buff.rvecs, _buff.tvecs, false, cv::SOLVEPNP_IPPE_SQUARE
						// , cv::noArray(), cv::noArray(), rmse
					);

					for(size_t s = 0; s < _buff.tvecs.size(); s++) {
						frc::Transform3d c2tag = util::cvToWpi(_buff.tvecs[s], _buff.rvecs[s]);
						_buff.i_estimations.push_back(_global.aprilp_field_poses[iid].TransformBy(c2tag.Inverse()));
						//_buff.distances.push_back(c2tag.Translation().Norm().value());	// add constexpr param for inserting these
					}
					//_buff.eerrors.insert(_buff.eerrors.end(), rmse.begin(), rmse.end());
#endif

				}

#if MEGATAG_SOLVE_METHOD > -1
				cv::solvePnPGeneric(
					_buff.obj_points, _buff.img_points,
					target.camera_matrix, target.dist_matrix,
					_buff.rvecs, _buff.tvecs, false, MEGATAG_SOLVE_METHOD,
					// cv::noArray(), cv::noArray(), rmse
					cv::noArray(), cv::noArray(), _buff.eerrors
				);

				for(size_t i = 0; i < _buff.tvecs.size(); i++) {
					frc::Transform3d f2cam = util::cvToWpi(_buff.tvecs[i], _buff.rvecs[i]).Inverse();
					_buff.c_estimations.emplace_back(f2cam.Translation(), f2cam.Rotation());
					_buff.distances.insert(_buff.distances.begin() + i, 0.f);
					for(int32_t iid : _buff.tag_ids) {
						_buff.distances[i] += f2cam.Translation().Distance(_global.aprilp_field_poses[iid].Translation()).value();
					}
					_buff.distances[i] /= detections;
				}
				// _buff.eerrors.insert(_buff.eerrors.begin(), rmse.begin(), rmse.end());
#endif

			}
		}
		
		_global.vpp.april_profiling[1] = duration<float>(high_resolution_clock::now() - p).count();

		static constexpr size_t POSE3D_ARRLEN = (sizeof(frc::Pose3d) / sizeof(double));
		const int64_t ts = nt::Now();
		double* d = reinterpret_cast<double*>(_buff.i_estimations.data());
		_global.nt.i_poses.Set(std::span<double>(d, d + (_buff.i_estimations.size() * POSE3D_ARRLEN)), ts);
		d = reinterpret_cast<double*>(_buff.c_estimations.data());
		_global.nt.c_poses.Set(std::span<double>(d, d + (_buff.c_estimations.size() * POSE3D_ARRLEN)), ts);
		_global.nt.pose_rmse.Set(_buff.eerrors, ts);
		_global.nt.pose_distances.Set(_buff.distances, ts);
		nt::NetworkTableInstance::GetDefault().Flush();
	} else {
		_global.vpp.april_profiling[1] = 0.f;
	}

	_global.vpp.april_profiling[2] = duration<float>(high_resolution_clock::now() - beg).count();
	_global.nt.april_timings.Set(std::span<float>(_global.vpp.april_profiling.begin(), _global.vpp.april_profiling.end()));

	_global.vpp.april_link = 2;

}
void _retro_worker_inst(CThread& target, const cv::Mat* f) {

	_global.vpp.retro_link = 1;

	high_resolution_clock::time_point p, beg = high_resolution_clock::now();
	decltype(_global.vpp.rtbuff)& _buff = _global.vpp.rtbuff;

#if VPIPE_CONFIG_NT_OPTIONS > 0
	const uint8_t
		wst_alpha = _buff.nt_wst_alpha.Get(),
		wst_beta = _buff.nt_wst_beta.Get(),
		wst_thresh = _buff.nt_wst_thresh.Get();
	const float
		tape_min_aspect = _buff.nt_tape_lower_wh.Get(),
		tape_max_aspect = _buff.nt_tape_upper_wh.Get(),
		tape_fill_thresh = _buff.nt_tape_fill_thresh.Get(),
		tape_pix_thresh = _buff.nt_tape_pix_thresh.Get();
#define _DETECTION_BASE				Constant::VRetro::DETECTION_BASE
#define _WST_ALPHA					wst_alpha
#define _WST_BETA					wst_beta
#define _WST_THRESH					wst_thresh
#define _DECIMATE_FACTOR			Constant::VRetro::DECIMATE_FACTOR
#define _TAPE_LOWER_WH_RATIO		tape_min_aspect
#define _TAPE_UPPER_WH_RATIO		tape_max_aspect
#define _TAPE_RECT_FILL_THRESH		tape_fill_thresh
#define _TAPE_PIXEL_COUNT_THRESH	tape_pix_thresh
#else
#define _DETECTION_BASE				Constant::VRetro::DETECTION_BASE
#define _WST_ALPHA					Constant::VRetro::WST_ALPHA
#define _WST_BETA					Constant::VRetro::WST_BETA
#define _WST_THRESH					Constant::VRetro::WST_THRESH
#define _DECIMATE_FACTOR			Constant::VRetro::DECIMATE_FACTOR
#define _TAPE_LOWER_WH_RATIO		Constant::VRetro::TAPE_LOWER_WH_RATIO
#define _TAPE_UPPER_WH_RATIO		Constant::VRetro::TAPE_UPPER_WH_RATIO
#define _TAPE_RECT_FILL_THRESH		Constant::VRetro::TAPE_RECT_FILL_THRESH
#define _TAPE_PIXEL_COUNT_THRESH	Constant::VRetro::TAPE_PIXEL_COUNT_THRESH
#endif

	_buff.contours.clear();
	_buff.centers.clear();
	_buff.bboxes.clear();
#if ESTIMATE_TAPE_DISTANCES
	_buff.locations.clear();
#endif

	const cv::Mat* src = (f ? f : &target.vpipe.frame);
	if constexpr(_DECIMATE_FACTOR > 1.f) {
		const cv::Size2i dfsz = src->size() / _DECIMATE_FACTOR;
		if(_buff.binary.size() != dfsz) {
			_buff.binary = cv::Mat(dfsz, CV_8UC1);
		}
		if(_buff.decimate.size() != dfsz) {
			_buff.decimate = cv::Mat(dfsz, CV_8UC3);
		}
		cv::resize(*src, _buff.decimate, dfsz, 0.0, 0.0, cv::INTER_AREA);
		src = &_buff.decimate;
	} else {
		if(_buff.binary.size() != src->size()) {
			_buff.binary = cv::Mat(src->size(), CV_8UC1);
		}
	}
	_global.vpp.retro_profiling[0] = duration<float>(high_resolution_clock::now() - beg).count();

	p = high_resolution_clock::now();
	neon_deinterlace_wstb(*src, _buff.binary, _DETECTION_BASE, _WST_ALPHA, _WST_BETA, _WST_THRESH);
	_global.vpp.retro_profiling[1] = duration<float>(high_resolution_clock::now() - p).count();

	p = high_resolution_clock::now();
	cv::findContours(_buff.binary, _buff.contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	_global.vpp.retro_profiling[2] = duration<float>(high_resolution_clock::now() - p).count();

	p = high_resolution_clock::now();
	for(size_t i = 0; i < _buff.contours.size(); i++) {
		std::vector<cv::Point2i>& contour = _buff.contours[i];
		float src_area = cv::contourArea(contour), bbox_area = 0.f;
		bool ratio{false}, fill{false}, area{src_area > (_TAPE_PIXEL_COUNT_THRESH / _DECIMATE_FACTOR)};
		cv::Rect2i bbox;
		if constexpr(Constant::VRetro::FILTER_UPRIGHT) {
			bbox = cv::boundingRect(contour);
			ratio = ::inRange((float)bbox.width / (float)bbox.height, _TAPE_LOWER_WH_RATIO, _TAPE_UPPER_WH_RATIO);
			bbox_area = bbox.area();
		} else {
			cv::RotatedRect outline = cv::minAreaRect(contour);
			ratio = ::inRange<double>(outline.size.aspectRatio(), _TAPE_LOWER_WH_RATIO, _TAPE_UPPER_WH_RATIO);
			bbox_area = outline.size.area();
			bbox = outline.boundingRect();
		}
		fill = (src_area / bbox_area) >= _TAPE_RECT_FILL_THRESH;
		if(ratio && fill && area) {
			size_t insert = _buff.bboxes.size();
			for(size_t x = 0; x < _buff.bboxes.size(); x++) {
				if(bbox_area > _buff.bboxes[x].area()) {
					insert = x;
					break;
				}
			}
			_buff.bboxes.insert(_buff.bboxes.begin() + insert, bbox);
			_buff.centers.insert(_buff.centers.begin() + insert, (cv::Point2d(bbox.tl() + bbox.br()) / 2.0));
			_buff.centers.at(insert).x /= _buff.binary.size().width;
			_buff.centers.at(insert).y /= _buff.binary.size().height;
#if ESTIMATE_TAPE_DISTANCES
			std::array<cv::Point2f, 4> _img;
			std::array<cv::Point3f, 4> _obj;
			_img[0] = bbox.tl() * _DECIMATE_FACTOR;
			_img[1] = cv::Point2f(bbox.x + bbox.width, bbox.y) * _DECIMATE_FACTOR;
			_img[2] = bbox.br() * _DECIMATE_FACTOR;
			_img[3] = cv::Point2f(bbox.x, bbox.y + bbox.height) * _DECIMATE_FACTOR;
			const float
				_yc = Constant::VRetro::TAPE_HEIGHT_INCHES / 2.f,
				_xc = _yc * bbox.size().aspectRatio();
			_obj[0] = cv::Point3f(-_xc, _yc, 0.f);
			_obj[1] = cv::Point3f(_xc, _yc, 0.f);
			_obj[2] = cv::Point3f(_xc, -_yc, 0.f);
			_obj[3] = cv::Point3f(-_xc, -_yc, 0.f);

			cv::solvePnP(_obj, _img,
				target.camera_matrix, target.dist_matrix,
				_buff.rvec, _buff.tvec,
				false, cv::SOLVEPNP_IPPE_SQUARE
			);
			// frc::Transform3d c2tape = util::cvToWpi(_buff.tvec, _buff.rvec);
			_buff.locations.insert(_buff.locations.begin() + insert, util::cvToWpi_T(_buff.tvec));
#endif
		}
	}
	_global.vpp.retro_profiling[3] = duration<float>(high_resolution_clock::now() - p).count();
	if(_buff.centers.size() > 0) {
		double* d = reinterpret_cast<double*>(_buff.centers.data());
		_global.nt.nodes.Set(std::span<double>(d, d + (_buff.centers.size() * sizeof(cv::Point2d) / sizeof(double))));
#if ESTIMATE_TAPE_DISTANCES
		d = reinterpret_cast<double*>(_buff.locations.data());
		_global.nt.nodes_3d.Set(std::span<double>(d, d + (_buff.locations.size() * sizeof(frc::Translation3d) / sizeof(double))));
#endif
		nt::NetworkTableInstance::GetDefault().Flush();
	}

	_global.vpp.retro_profiling[4] = duration<float>(high_resolution_clock::now() - beg).count();
	_global.nt.retro_timings.Set(std::span<float>(_global.vpp.retro_profiling.begin(), _global.vpp.retro_profiling.end()));

	_global.vpp.retro_link = 2;

}

void _april_worker(CThread& target, const cv::Mat* f) {

	_april_worker_inst(target, f);		// may convert this into a loop with state checking later

}
void _retro_worker(CThread& target, const cv::Mat* f) {

	_retro_worker_inst(target, f);		// '''

}
