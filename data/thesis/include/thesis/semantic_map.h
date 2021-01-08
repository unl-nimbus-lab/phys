/**
 * @author Carsten Könemann
 */
#ifndef __SEMANTIC_MAP__
#define __SEMANTIC_MAP__

#include <thesis/ObjectInstance.h>

#include <thesis/uuid.h>

#include <opencv2/core/core.hpp>

#include <tf/tf.h>

#include <deque>

class SemanticMap
{
  public:
    /**
     * Constructor.
     */
    SemanticMap(const int memory_size=0, const bool debug=true) : memory_size(memory_size), debug(debug)
    {
      // Starting position
      currentPosition.x = 0;
      currentPosition.y = 0;
      currentPosition.z = 0;
    };
    
    /**
     * Default destructor.
     */
    ~SemanticMap()
    {
      //
    };
    
    /**
     * @param Check if an entry for this ID exists in the database.
     */
    bool exists(std::string id);
    
    /**
     * Needed to evaluate an entry by distance.
     *
     * @param Use the current camera position.
     */
    void setCurrentPosition(cv::Point3f p);
    
    /**
     * Flag an object for removal.
     */
    void flag(const std::string& type,
              const boost::uuids::uuid& id,
              double age_threshold);
    
    /**
     * Attempt to remove false positives.
     */
    void cleanup(double age_threshold, unsigned int min_confirmations);

    // Getters
    void getAll(std::vector<thesis::ObjectInstance>& out);
    
    void getByID(std::string id, std::vector<thesis::ObjectInstance>& out);
    
    void getByIDAtPosition(std::string id,
                           cv::Point3f p,
                           std::vector<thesis::ObjectInstance>& out,
                           float max_distance=0.0f);
                           
    void getByPosition(cv::Point3f p,
                       std::vector<thesis::ObjectInstance>& out,
                       float max_distance=0.0f);
    
    
    
    /**
     * @param  Add this object to the storage of this map.
     * @param  The UUID of the updated object (if no new one was added).
     * @param  Minimal distance to existing objects,
     *         otherwise attempt updating instead of adding.
     * @return true, if an object was added,
     *         false, if an existing one was updated instead.
     */
    bool add(const thesis::ObjectInstance& object,
             boost::uuids::uuid& id,
             float min_distance=0.0f);
  
  protected:
    class ObjectQueue
    {
      public:
        /**
         * Constructor.
         */
        ObjectQueue(const int memory_size=0) : memory_size(memory_size)
        {
          // Initialize UUID
          id = uuid_msgs::random();
          // Remember time of initialization
          stamp_init = ros::Time::now();
          stamp_flag = stamp_init;
          // Not yet flagged for removal
          flags = 0;
          // For debugging purposes
          ROS_DEBUG("SemanticMap::ObjectQueue(%i);", memory_size);
        };
        
        /**
         * Default destructor.
         */
        ~ObjectQueue()
        {
          //
        };
        
        /**
         * @return This objects UUID.
         */
        boost::uuids::uuid getID();
        
        /**
         * @return Time since initialization in seconds.
         */
        double age();
        
        /**
         * @return Number of instances currently stored in this queue.
         */
        size_t size();
        
        /**
         * @return Weighted average of all values contained in this queue.
         */
        thesis::ObjectInstance combined();
        
        /**
         * @param Add object and remove oldest entry (if exceeding max size).
         */
        void add(thesis::ObjectInstance o);
        
        /**
         * Flag for removal.
         */
        void flag(double age_threshold);
        
        /**
         * @return Number of times this instance is flagged for removal.
         */
        unsigned int flagged();
        
      protected:
        /**
         * Universally unique identifier.
         */
        boost::uuids::uuid id;
      
        /**
         * Using a std::deque as a base to expand upon.
         */
        std::deque<thesis::ObjectInstance> deque;
        
        /**
         * Maximum number of values to store for averaging.
         */
        int memory_size;
        
        /**
         * Counts how many times this instance was flagged for removal.
         */
        unsigned int flags;
        
        /**
         * Time of initialization.
         */
        ros::Time stamp_init,
                  stamp_flag;
    };

    struct EvaluationComparator
    {
      public:
        /** 
         * non-static Comparator needs to know its containing object.
         */
        SemanticMap* containingObject;
        
        /**
         * Constructor.
         */
        EvaluationComparator(SemanticMap* o) : containingObject(o)
        {
          //
        }
      
        /**
         * Compare-function.
         */
        bool operator() (const thesis::ObjectInstance& a,
                         const thesis::ObjectInstance& b)
        {
          return evaluate(a, containingObject->currentPosition)
               < evaluate(b, containingObject->currentPosition);
        }
      
      protected:
        /**
         * Evaluation-function for perceived objects.
         * Used to sort perceived objects by relevance.
         */
        double evaluate(const thesis::ObjectInstance& object, cv::Point3f p_c);
    };
  
    /**
     * Hold all elements of the database, with fast access to entrys by ID.
     */
    std::map<std::string, std::vector<ObjectQueue> > map;
    
    /**
     * Maximum number of values to store per entry for averaging.
     */
    int memory_size;
    
    /**
     *
     */
    cv::Point3f currentPosition;
    
    /**
     *
     */
    bool debug;
};

#endif //__SEMANTIC_MAP__
