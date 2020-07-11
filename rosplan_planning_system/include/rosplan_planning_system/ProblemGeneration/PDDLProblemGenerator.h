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
        typedef std::vector<rosplan_knowledge_msgs::DomainFormula> DomainFormulas;
        
        bool preparePredicateParameters(DomainFormulas &domainFormulas, 
                                        rosplan_knowledge_msgs::KnowledgeItem &kwItem,
                                        std::ofstream &pFile);
        
        bool writeDomainName(std::ofstream &pFile);
        bool writeObjects(std::ofstream &pFile);        
        bool getPropositions(ros::Time &time, std::ofstream &pFile);
        bool getFunctions(ros::Time &time, std::ofstream &pFile);
        void printExpression(std::ofstream &pFile, rosplan_knowledge_msgs::ExprComposite &e);
     
        bool makeHeader(std::ofstream &pFile);
        bool makeInitialState(std::ofstream &pFile);
        bool makeGoals(std::ofstream &pFile);
        void makeMetric(std::ofstream &pFile);
        void makeFooter(std::ofstream &pFile);
        
        void makeProblem(std::ofstream &pFile);
        
    public:
        PDDLProblemGenerator(const std::string& kb);
    };
} // close namespace

#endif
