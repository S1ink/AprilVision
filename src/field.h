#pragma once

#include <array>

#include <frc/geometry/Pose3d.h>

#include <core/aprilpose.h>



static inline constexpr float
	TAG_SIDE_INCHES = 6.5f;		// 6.5 inner dimension, was previously 6.0 for 2023
static inline const std::array<cv::Point3f, 4>
	GENERIC_TAG_CORNERS{	// in camera coord system
		cv::Point3f{ -TAG_SIDE_INCHES / 2.f, TAG_SIDE_INCHES / 2.f, 0.f },
		cv::Point3f{ TAG_SIDE_INCHES / 2.f, TAG_SIDE_INCHES / 2.f, 0.f },
		cv::Point3f{ TAG_SIDE_INCHES / 2.f, -TAG_SIDE_INCHES / 2.f, 0.f },
		cv::Point3f{ -TAG_SIDE_INCHES / 2.f, -TAG_SIDE_INCHES / 2.f, 0.f }
	};


// 2024: https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf (page 4)

static inline cv::Ptr<cv::aruco::Board> const
	FIELD_2024{ cv::aruco::Board::create(
		// calculations and formatting: https://docs.google.com/spreadsheets/d/1obM2zp_ohWkl_-QUPi7S-PNwsVBvjXx0UC8woM7FiGU/edit?usp=sharing
		std::vector<std::vector<cv::Point3f>>{ {
			{
				cv::Point3f(596.494582562299,11.305,56.63),
				cv::Point3f(590.865417437701,8.055,56.63),
				cv::Point3f(590.865417437701,8.055,50.13),
				cv::Point3f(596.494582562299,11.305,50.13)
			}, {
				cv::Point3f(640.024582562299,36.415,56.63),
				cv::Point3f(634.395417437701,33.165,56.63),
				cv::Point3f(634.395417437701,33.165,50.13),
				cv::Point3f(640.024582562299,36.415,50.13)
			}, {
				cv::Point3f(652.73,199.42,60.38),
				cv::Point3f(652.73,192.92,60.38),
				cv::Point3f(652.73,192.92,53.88),
				cv::Point3f(652.73,199.42,53.88)
			}, {
				cv::Point3f(652.73,221.67,60.38),
				cv::Point3f(652.73,215.17,60.38),
				cv::Point3f(652.73,215.17,53.88),
				cv::Point3f(652.73,221.67,53.88)
			}, {
				cv::Point3f(575.52,323,56.63),
				cv::Point3f(582.02,323,56.63),
				cv::Point3f(582.02,323,50.13),
				cv::Point3f(575.52,323,50.13)
			}, {
				cv::Point3f(69.25,323,56.63),
				cv::Point3f(75.75,323,56.63),
				cv::Point3f(75.75,323,50.13),
				cv::Point3f(69.25,323,50.13)
			}, {
				cv::Point3f(-1.5,215.17,60.38),
				cv::Point3f(-1.5,221.67,60.38),
				cv::Point3f(-1.5,221.67,53.88),
				cv::Point3f(-1.5,215.17,53.88)
			}, {
				cv::Point3f(-1.5,192.92,60.38),
				cv::Point3f(-1.5,199.42,60.38),
				cv::Point3f(-1.5,199.42,53.88),
				cv::Point3f(-1.5,192.92,53.88)
			}, {
				cv::Point3f(16.8345825622994,33.165,56.63),
				cv::Point3f(11.2054174377006,36.415,56.63),
				cv::Point3f(11.2054174377006,36.415,50.13),
				cv::Point3f(16.8345825622994,33.165,50.13)
			}, {
				cv::Point3f(60.3545825622994,8.055,56.63),
				cv::Point3f(54.7254174377006,11.305,56.63),
				cv::Point3f(54.7254174377006,11.305,50.13),
				cv::Point3f(60.3545825622994,8.055,50.13)
			}, {
				cv::Point3f(465.875417437701,144.565,55.25),
				cv::Point3f(471.504582562299,147.815,55.25),
				cv::Point3f(471.504582562299,147.815,48.75),
				cv::Point3f(465.875417437701,144.565,48.75)
			}, {
				cv::Point3f(471.504582562299,175.475,55.25),
				cv::Point3f(465.875417437701,178.725,55.25),
				cv::Point3f(465.875417437701,178.725,48.75),
				cv::Point3f(471.504582562299,175.475,48.75)
			}, {
				cv::Point3f(441.74,164.87,55.25),
				cv::Point3f(441.74,158.37,55.25),
				cv::Point3f(441.74,158.37,48.75),
				cv::Point3f(441.74,164.87,48.75)
			}, {
				cv::Point3f(209.48,158.37,55.25),
				cv::Point3f(209.48,164.87,55.25),
				cv::Point3f(209.48,164.87,48.75),
				cv::Point3f(209.48,158.37,48.75)
			}, {
				cv::Point3f(185.544582562299,178.725,55.25),
				cv::Point3f(179.915417437701,175.475,55.25),
				cv::Point3f(179.915417437701,175.475,48.75),
				cv::Point3f(185.544582562299,178.725,48.75)
			}, {
				cv::Point3f(179.915417437701,147.815,55.25),
				cv::Point3f(185.544582562299,144.565,55.25),
				cv::Point3f(185.544582562299,144.565,48.75),
				cv::Point3f(179.915417437701,147.815,48.75)
			}
		} },
		cv::aruco::getPredefinedDictionary(FRC_DICT),
		std::array<int32_t, 16>{
			1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16
		}
	) };

static inline std::array<frc::Pose3d, 16> const
	TAG_POSES_2024{
		frc::Pose3d{ frc::Translation3d{593.68_in, 9.68_in, 53.38_in},		frc::Rotation3d{ frc::Quaternion{0.5, 0.0, 0.0, 0.8660254} } },
		frc::Pose3d{ frc::Translation3d{637.21_in, 34.79_in, 53.38_in},		frc::Rotation3d{ frc::Quaternion{0.5, 0.0, 0.0, 0.8660254} } },
		frc::Pose3d{ frc::Translation3d{652.73_in, 196.17_in, 57.13_in},	frc::Rotation3d{ frc::Quaternion{0.0, 0.0, 0.0, 1.0} } },
		frc::Pose3d{ frc::Translation3d{652.73_in, 218.42_in, 57.13_in},	frc::Rotation3d{ frc::Quaternion{0.0, 0.0, 0.0, 1.0} } },
		frc::Pose3d{ frc::Translation3d{578.77_in, 323_in, 53.38_in},		frc::Rotation3d{ frc::Quaternion{-0.7071068, 0.0, 0.0, 0.7071068} } },
		frc::Pose3d{ frc::Translation3d{72.5_in, 323_in, 53.38_in},			frc::Rotation3d{ frc::Quaternion{-0.7071068, 0.0, 0.0, 0.7071068} } },
		frc::Pose3d{ frc::Translation3d{-1.5_in, 218.42_in, 57.13_in},		frc::Rotation3d{ frc::Quaternion{1.0, 0.0, 0.0, 0.0} } },
		frc::Pose3d{ frc::Translation3d{-1.5_in, 196.17_in, 57.13_in},		frc::Rotation3d{ frc::Quaternion{1.0, 0.0, 0.0, 0.0} } },
		frc::Pose3d{ frc::Translation3d{14.02_in, 34.79_in, 53.38_in},		frc::Rotation3d{ frc::Quaternion{0.8660254, 0.0, 0.0, 0.5} } },
		frc::Pose3d{ frc::Translation3d{57.54_in, 9.68_in, 53.38_in},		frc::Rotation3d{ frc::Quaternion{0.8660254, 0.0, 0.0, 0.5} } },
		frc::Pose3d{ frc::Translation3d{468.69_in, 146.19_in, 52_in},		frc::Rotation3d{ frc::Quaternion{-0.8660254, 0.0, 0.0, 0.5} } },
		frc::Pose3d{ frc::Translation3d{468.69_in, 177.1_in, 52_in},		frc::Rotation3d{ frc::Quaternion{0.8660254, 0.0, 0.0, 0.5} } },
		frc::Pose3d{ frc::Translation3d{441.74_in, 161.62_in, 52_in},		frc::Rotation3d{ frc::Quaternion{0.0, 0.0, 0.0, 1.0} } },
		frc::Pose3d{ frc::Translation3d{209.48_in, 161.62_in, 52_in},		frc::Rotation3d{ frc::Quaternion{1.0, 0.0, 0.0, 0.0} } },
		frc::Pose3d{ frc::Translation3d{182.73_in, 177.1_in, 52_in},		frc::Rotation3d{ frc::Quaternion{0.5, 0.0, 0.0, 0.8660254} } },
		frc::Pose3d{ frc::Translation3d{182.73_in, 146.19_in, 52_in},		frc::Rotation3d{ frc::Quaternion{-0.5, 0.0, 0.0, 0.8660254} } }
	};





// 2023: https://firstfrc.blob.core.windows.net/frc2023/FieldAssets/2023LayoutMarkingDiagram.pdf (page 4)

static inline cv::Ptr<cv::aruco::Board> const	// the frc 2023 field has 8 markers - source is linked above
	FIELD_2023{ cv::aruco::Board::create(
		// calculations and formatting: https://docs.google.com/spreadsheets/d/1c-rZgIMjbx5Ruouc-R8gldkDkMjAIx7uA9ZRElyW1K4/edit?usp=sharing
		std::vector<std::vector<cv::Point3f>>{ {
			{
				cv::Point3f(610.77,39.19,15.22),
				cv::Point3f(610.77,45.19,15.22),
				cv::Point3f(610.77,45.19,21.22),
				cv::Point3f(610.77,39.19,21.22)
			}, {
				cv::Point3f(610.77,105.19,15.22),
				cv::Point3f(610.77,111.19,15.22),
				cv::Point3f(610.77,111.19,21.22),
				cv::Point3f(610.77,105.19,21.22)
			}, {
				cv::Point3f(610.77,171.19,15.22),
				cv::Point3f(610.77,177.19,15.22),
				cv::Point3f(610.77,177.19,21.22),
				cv::Point3f(610.77,171.19,21.22)
			}, {
				cv::Point3f(636.96,262.74,24.38),
				cv::Point3f(636.96,268.74,24.38),
				cv::Point3f(636.96,268.74,30.38),
				cv::Point3f(636.96,262.74,30.38)
			}, {
				cv::Point3f(14.25,268.74,24.38),
				cv::Point3f(14.25,262.74,24.38),
				cv::Point3f(14.25,262.74,30.38),
				cv::Point3f(14.25,268.74,30.38)
			}, {
				cv::Point3f(40.45,177.19,15.22),
				cv::Point3f(40.45,171.19,15.22),
				cv::Point3f(40.45,171.19,21.22),
				cv::Point3f(40.45,177.19,21.22)
			}, {
				cv::Point3f(40.45,111.19,15.22),
				cv::Point3f(40.45,105.19,15.22),
				cv::Point3f(40.45,105.19,21.22),
				cv::Point3f(40.45,111.19,21.22)
			}, {
				cv::Point3f(40.45,45.19,15.22),
				cv::Point3f(40.45,39.19,15.22),
				cv::Point3f(40.45,39.19,21.22),
				cv::Point3f(40.45,45.19,21.22)
			}
		} },
		cv::aruco::getPredefinedDictionary(FRC_DICT_2023),
		std::array<int32_t, 8>{
			1, 2, 3, 4, 5, 6, 7, 8
		}
	) };

static inline std::array<frc::Pose3d, 8> const
	TAG_POSES_2023{
		frc::Pose3d( frc::Translation3d(610.77_in, 42.19_in, 18.22_in),		frc::Rotation3d(frc::Quaternion(0.0, 0.0, 0.0, 1.0)) ),
		frc::Pose3d( frc::Translation3d(610.77_in, 108.19_in, 18.22_in),	frc::Rotation3d(frc::Quaternion(0.0, 0.0, 0.0, 1.0)) ),
		frc::Pose3d( frc::Translation3d(610.77_in, 174.19_in, 18.22_in),	frc::Rotation3d(frc::Quaternion(0.0, 0.0, 0.0, 1.0)) ),
		frc::Pose3d( frc::Translation3d(636.96_in, 265.74_in, 27.38_in),	frc::Rotation3d(frc::Quaternion(0.0, 0.0, 0.0, 1.0)) ),
		frc::Pose3d( frc::Translation3d(14.25_in, 265.74_in, 27.38_in),		frc::Rotation3d(frc::Quaternion(1.0, 0.0, 0.0, 0.0)) ),
		frc::Pose3d( frc::Translation3d(40.45_in, 174.19_in, 18.22_in),		frc::Rotation3d(frc::Quaternion(1.0, 0.0, 0.0, 0.0)) ),
		frc::Pose3d( frc::Translation3d(40.45_in, 108.19_in, 18.22_in),		frc::Rotation3d(frc::Quaternion(1.0, 0.0, 0.0, 0.0)) ),
		frc::Pose3d( frc::Translation3d(40.45_in, 42.19_in, 18.22_in),		frc::Rotation3d(frc::Quaternion(1.0, 0.0, 0.0, 0.0)) )
	};





// 2022 (unofficial)

// https://docs.opencv.org/4.5.2/db/da9/tutorial_aruco_board_detection.html
// https://docs.google.com/document/d/e/2PACX-1vQxVFxsY30_6sy50N8wWUpUhQ0qbUKnw7SjW6agbKQZ2X0SN_uXtNZhLB7AkRcJjLnlcmmjcyCNhn0I/pub
// https://docs.google.com/document/d/e/2PACX-1vQxVFxsY30_6sy50N8wWUpUhQ0qbUKnw7SjW6agbKQZ2X0SN_uXtNZhLB7AkRcJjLnlcmmjcyCNhn0I/pub
// https://docs.google.com/document/d/e/2PACX-1vSizkGFRocq8-QLCj38O68MO4wYCThk_z60g7KhBLf497UqnLHcLW9r1HcTKzwI_SoYLHZp7wPnU6H4/pub

static inline cv::Ptr<cv::aruco::Board> const	// the 2022 (unofficial) field has 24 markers - link to source above
	FIELD_2022{ cv::aruco::Board::create(
		// calculations and formatting: https://docs.google.com/spreadsheets/d/1zpj37KxVP_r6md_0VjLPmQ9h4aVExXe4bx78hdGrro8/edit?usp=sharing
		std::vector<std::vector<cv::Point3f>>{ {
			{
				cv::Point3f(-0.139,295.133,38.126),
				cv::Point3f(-0.139,301.633,38.126),
				cv::Point3f(-0.139,301.633,31.626),
				cv::Point3f(-0.139,295.133,31.626)
			}, {
				cv::Point3f(127.272,212.76,71.182),
				cv::Point3f(127.272,219.26,71.182),
				cv::Point3f(127.272,219.26,64.682),
				cv::Point3f(127.272,212.76,64.682)
			}, {
				cv::Point3f(117.53,209.863,57.432),
				cv::Point3f(124.03,209.863,57.432),
				cv::Point3f(124.03,209.863,50.932),
				cv::Point3f(117.53,209.863,50.932)
			}, {
				cv::Point3f(0.157,195.905,35),
				cv::Point3f(0.157,202.405,35),
				cv::Point3f(0.157,202.405,28.5),
				cv::Point3f(0.157,195.905,28.5)
			}, {
				cv::Point3f(0.157,135.037,35),
				cv::Point3f(0.157,141.537,35),
				cv::Point3f(0.157,141.537,28.5),
				cv::Point3f(0.157,135.037,28.5)
			}, {
				cv::Point3f(7.11568287669421,65.3835825687076,38.313),
				cv::Point3f(2.42031712330579,69.8784174312924,38.313),
				cv::Point3f(2.42031712330579,69.8784174312924,31.813),
				cv::Point3f(7.11568287669421,65.3835825687076,31.813)
			}, {
				cv::Point3f(36.7296828766942,34.8115825687076,38.313),
				cv::Point3f(32.0343171233058,39.3064174312924,38.313),
				cv::Point3f(32.0343171233058,39.3064174312924,31.813),
				cv::Point3f(36.7296828766942,34.8115825687076,31.813)
			}, {
				cv::Point3f(65.9336828766942,3.94358256870762,38.313),
				cv::Point3f(61.2383171233058,8.43841743129238,38.313),
				cv::Point3f(61.2383171233058,8.43841743129238,31.813),
				cv::Point3f(65.9336828766942,3.94358256870762,31.813)
			}, {
				cv::Point3f(648.139,28.867,38.126),
				cv::Point3f(648.139,22.367,38.126),
				cv::Point3f(648.139,22.367,31.626),
				cv::Point3f(648.139,28.867,31.626)
			}, {
				cv::Point3f(521.063,111.26,71.182),
				cv::Point3f(521.063,104.76,71.182),
				cv::Point3f(521.063,104.76,64.682),
				cv::Point3f(521.063,111.26,64.682)
			}, {
				cv::Point3f(530.47,114.167,57.432),
				cv::Point3f(523.97,114.167,57.432),
				cv::Point3f(523.97,114.167,50.932),
				cv::Point3f(530.47,114.167,50.932)
			}, {
				cv::Point3f(647.843,128.27,35),
				cv::Point3f(647.843,121.77,35),
				cv::Point3f(647.843,121.77,28.5),
				cv::Point3f(647.843,128.27,28.5)
			}, {
				cv::Point3f(647.843,188.964,35),
				cv::Point3f(647.843,182.464,35),
				cv::Point3f(647.843,182.464,28.5),
				cv::Point3f(647.843,188.964,28.5)
			}, {
				cv::Point3f(640.861534684921,258.84072074132,38.438),
				cv::Point3f(645.360465315079,254.14927925868,38.438),
				cv::Point3f(645.360465315079,254.14927925868,31.938),
				cv::Point3f(640.861534684921,258.84072074132,31.938)
			}, {
				cv::Point3f(611.549534684921,289.45972074132,38.313),
				cv::Point3f(616.048465315079,284.76827925868,38.313),
				cv::Point3f(616.048465315079,284.76827925868,31.813),
				cv::Point3f(611.549534684921,289.45972074132,31.813)
			}, {
				cv::Point3f(582.285534684921,320.02772074132,38.313),
				cv::Point3f(586.784465315079,315.33627925868,38.313),
				cv::Point3f(586.784465315079,315.33627925868,31.813),
				cv::Point3f(582.285534684921,320.02772074132,31.813)
			}, {
				cv::Point3f(312.974022737338,194.753894089996,30.938),
				cv::Point3f(307.035977262662,192.110105910004,30.938),
				cv::Point3f(307.035977262662,192.110105910004,24.438),
				cv::Point3f(312.974022737338,194.753894089996,24.438)
			}, {
				cv::Point3f(291.246105910004,150.974022737338,30.938),
				cv::Point3f(293.889894089996,145.035977262662,30.938),
				cv::Point3f(293.889894089996,145.035977262662,24.438),
				cv::Point3f(291.246105910004,150.974022737338,24.438)
			}, {
				cv::Point3f(335.025977262662,129.246105910004,30.938),
				cv::Point3f(340.964022737338,131.889894089996,30.938),
				cv::Point3f(340.964022737338,131.889894089996,24.438),
				cv::Point3f(335.025977262662,129.246105910004,24.438)
			}, {
				cv::Point3f(356.753894089996,173.025977262662,30.938),
				cv::Point3f(354.110105910004,178.964022737338,30.938),
				cv::Point3f(354.110105910004,178.964022737338,24.438),
				cv::Point3f(356.753894089996,173.025977262662,24.438)
			}, {
				cv::Point3f(302.123035778737,173.879364166192,98.0881815660862),
				cv::Point3f(299.793644106692,167.81109139396,98.0881815660862),
				cv::Point3f(302.524964221264,166.762635833808,92.2838184339138),
				cv::Point3f(304.854355893308,172.83090860604,92.2838184339138)
			}, {
				cv::Point3f(312.120635833808,140.123035778737,98.0881815660862),
				cv::Point3f(318.18890860604,137.793644106692,98.0881815660862),
				cv::Point3f(319.237364166192,140.524964221264,92.2838184339138),
				cv::Point3f(313.16909139396,142.854355893308,92.2838184339138)
			}, {
				cv::Point3f(345.876964221264,150.120635833808,98.0881815660862),
				cv::Point3f(348.206355893308,156.18890860604,98.0881815660862),
				cv::Point3f(345.475035778737,157.237364166192,92.2838184339138),
				cv::Point3f(343.145644106692,151.169091393961,92.2838184339138)
			}, {
				cv::Point3f(335.879364166192,183.876964221263,98.0881815660862),
				cv::Point3f(329.811091393961,186.206355893308,98.0881815660862),
				cv::Point3f(328.762635833808,183.475035778737,92.2838184339138),
				cv::Point3f(334.83090860604,181.145644106692,92.2838184339138)
			}
		} },
		cv::aruco::getPredefinedDictionary(FRC_DICT),
		std::array<int32_t, 24>{
			0, 1, 2, 3, 4, 5, 6, 7,
			10, 11, 12, 13, 14, 15, 16, 17,
			40, 41, 42, 43,
			50, 51, 52, 53
		}
	) };