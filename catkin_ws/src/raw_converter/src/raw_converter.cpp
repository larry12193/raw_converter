#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/program_options.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <fstream>
#include <sstream>

#define BOLDBLUE "\033[1m\033[94m"
#define BOLDRED  "\033[1m\033[31m"
#define ENDC   "\033[0m"

using namespace std;
namespace po = boost::program_options;

void thread(string, string, string, string);
int MAX_NUM_OF_THREADS = 6;

void thread(std::string file_name, std::string raw_path, std::string out_path) {

  stringstream fname;
  // Filename without extension
  std::string rawName;
  // Find index of period before extension
  size_t lastIndex = file_name.find_last_of(".");
  // Remove file extension to get raw filename
  rawName = file_name.substr(0,lastIndex);

  // Construct output path with new filename
  fname << out_path << "/" << rawName << ".jpg";

  // Define raw image full path
  std::string totalRawPath = raw_path + "/" + file_name;

  // Load raw image
  ifstream fs;
  fs.open(totalRawPath.c_str(), ios::in | ios::binary );
  unsigned short *tmp = new unsigned short [640*480];
  fs.read((char*)tmp, 640*480*2);
  fs.close();

  // Define image matrix object
  cv::Mat image = cv::Mat(480,640, CV_16UC1,tmp);
  normalize(image, image, 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );

  // Save as jpg
  cv::imwrite(fname.str().c_str(), image);
  return;
}

int main( int argc, char** argv ) {

    // Parse input arguments
    po::options_description desc("Allowed Options");
    desc.add_options()
        ("help,h", "Allowed Options:")
        ("dir,d", po::value<string>(), "The path to a directory of files to be processed.");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if(vm.count("help")) {
        cout << desc << "\n";
        return 1;
    }

    po::notify(vm);

    if(!vm.count("dir")) {
        cout << "No raw image directory supplied!.\n";
        cout << desc << "\n";
        return 1;
    }

    string raw_path = vm["dir"].as<string>();
    string out_path = raw_path + "/jpgs";

    vector<std::string> fnames; // File names in raw directory
    boost::filesystem::directory_iterator di(raw_path.c_str());
    boost::filesystem::directory_iterator di_end;

    // Load all filenames from raw source directory
    while( di != di_end ) {
      fnames.push_back(di->path().filename().string());
      ++di;
    }

    cout << BOLDBLUE << "Processing " << fnames.size() << " images in " << raw_path << ENDC << endl;
    cout << BOLDBLUE << "Placing output in " << out_path << ENDC << endl;

    char yes_no;
    boost::filesystem::path out_dir(out_path.c_str());
    if(boost::filesystem::exists(out_dir)) {
    //    cout << "Output directory already exists. Overwrite? (y/n): ";
    //    cin.get(yes_no);
    //    if( yes_no == 'y' || yes_no == 'Y' ) {
            cout << "Removing " << out_path.c_str() << endl;
            boost::filesystem::remove_all(out_path);
    //    }
    //    else {
    //        cout << "Quitting." << endl;
    //        return 1;
    //    }
    }

    cout << "Creating " << out_path.c_str() << endl;
    boost::filesystem::create_directory(out_dir);

    boost::thread_group threads;
    int count = 0;
    BOOST_FOREACH(std::string fn, fnames) {
      //threads.create_thread(boost::bind(thread, boost::cref(fn), boost::cref(raw_path), boost::cref(out_path)));
      thread(fn,raw_path, out_path);
      cout << count << ":" << fn << endl;
      // Don't start more than a set number of threads
      // TODO: Set the number of threads dynamically.
        // As one thread dies another should be immediately spawned.
        // See this as possible solution:
          // https://stackoverflow.com/questions/22685176/boost-group-threads-maximal-number-of-parallel-thread
      // if (count % MAX_NUM_OF_THREADS == 0) {
      //     threads.join_all();
      //     count = 0;
      // }
      // count++;
    }

    // threads.join_all();

}
