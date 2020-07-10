/**
 * This class is responsible for generating the PDDL instance.
 * This is done by using the objects requested from Knowedge services.
 */
#ifndef KCL_PDDLproblemgenerator
#define KCL_PDDLproblemgenerator

#include "ros/ros.h"
#include "ProblemGenerator.h"

namespace KCL_rosplan {

    class PDDLProblemGenerator : public ProblemGenerator 
    {
    private:        
        void getPropositions(ros::Time &time, std::ofstream &pFile);
        void getFunctions(ros::Time &time, std::ofstream &pFile);
        void printExpression(std::ofstream &pFile, rosplan_knowledge_msgs::ExprComposite &e);
     
        void makeHeader(std::ofstream &pFile);
        void makeInitialState(std::ofstream &pFile);
        void makeGoals(std::ofstream &pFile);
        void makeMetric(std::ofstream &pFile);
        void makeProblem(std::ofstream &pFile);
        
    public:
        PDDLProblemGenerator(const std::string& kb);
    };
} // close namespace

#endif
