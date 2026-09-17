/**
 * @author      : $USER ($USER@frkle-Predator-PT515-52)
 * @file        : stupid_colors
 * @created     : Tuesday Sep 15, 2026 15:48:02 UTC
 */

#ifndef STUPID_COLORS_H

#define STUPID_COLORS_H
#include <string>

const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");
#define ROS_YE(x) ROS_INFO_STREAM( yellow << x << reset)

const std::string bar("\n======================================================\n");


#endif /* end of include guard STUPID_COLORS_H */

