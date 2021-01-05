/**
 * @author Carsten Könemann
 */
#ifndef __DATABASE__
#define __DATABASE__

#include <thesis/ObjectClass.h>

#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>

#include <deque>

class Database
{
  public:
    /**
     * Default constructor.
     */
    Database(const int memory_size=0, const bool debug=false) : memory_size(memory_size), debug(debug)
    {
      //
    }
    
    /**
     * Default destructor.
     */
    ~Database()
    {
      //
    }

    /**
     * @param  Check if an entry for this ID exists in the database.
     *
     * @return true, if an entry for this ID exists in the database,
     *         false otherwise.
     */
    bool exists(std::string id);

    /**
     * @param  Retrieve an object from the database by this ID.
     * @param  The object belonging to this ID (output).
     *
     * @return true, if an entry for this ID exists in the database,
     *         false otherwise.
     */
    bool getByID(std::string id, thesis::ObjectClass& out);
    
    /**
     * @return All objects from the database.
     */
    std::vector<thesis::ObjectClass> getAll();
    
    /**
     *
     */
    bool update(const thesis::ObjectClass& input);
    
    /**
     * Add an OpenCV image to the database.
     *
     * @return true, if the image was added successfully,
     *         false otherwise.
     */
    bool add_image(cv::Mat&    image,
                   std::string name,
                   cv::Size    min_image_size,
                   cv::Size    max_image_size);
    
    /**
     * Add a ROS-message image to the database.
     *
     * @return true, if the image was added successfully,
     *         false otherwise.
     */
    bool add_image(sensor_msgs::Image& image,
                   std::string         name,
                   cv::Size            min_image_size,
                   cv::Size            max_image_size);
  
  protected:
    class ObjectQueue
    {
      public:
        /**
         * Default constructor.
         */
        ObjectQueue(const int memory_size=0, const bool debug=false) : memory_size(memory_size), debug(debug)
        {
          //
          ROS_DEBUG("Database::ObjectQueue(%i);", memory_size);
        }
        
        /**
         * Default destructor.
         */
        ~ObjectQueue()
        {
          //
        }
        
        /**
         * @return Weighted average of all values contained in this queue.
         */
        thesis::ObjectClass combined();
        
        /**
         * @param Add object and remove oldest entry (if exceeding max size).
         */
        void add(thesis::ObjectClass o);
        
      protected:
        /**
         * Using a std::deque as a base to expand upon.
         */
        std::deque<thesis::ObjectClass> deque;
        
        /**
         * Maximum number of values to store for averaging.
         */
        int memory_size;
        
        /**
         *
         */
        bool debug;
    };
  
    /**
     * A database entry.
     */
    struct Entry
    {
      /**
       * We need to keep the image one time only.
       */
      sensor_msgs::Image image;
      
      /**
       * Stores the most recent other values to calculate an average.
       */
      ObjectQueue values;
    };

    /**
     * Hold all elements of the database, with fast access to entrys by ID.
     */
    std::map<std::string, Entry> database;
    
    /**
     * Maximum number of values to store per entry for averaging.
     */
    int memory_size;
    
    /**
     *
     */
    bool debug;
};

#endif //__DATABASE__
