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
        
        bool printDomainName(std::ofstream &pFile);
        bool printObjects(std::ofstream &pFile);        
        bool printPredicateParameters(DomainFormulas &domainFormulas, 
                                      rosplan_knowledge_msgs::KnowledgeItem &kwItem, std::ofstream &pFile);
        bool printPropositions(ros::Time &time, std::ofstream &pFile);
        bool printFunctions(ros::Time &time, std::ofstream &pFile);
        void printExpression(std::ofstream &pFile, rosplan_knowledge_msgs::ExprComposite &e);
     
        bool makeHeader(std::ofstream &pFile);
        bool makeInitialState(std::ofstream &pFile);
        bool makeGoals(std::ofstream &pFile);
        bool makeMetric(std::ofstream &pFile);
        void makeFooter(std::ofstream &pFile);
        
        void makeProblem(std::ofstream &pFile);
        
    public:
        PDDLProblemGenerator(const std::string& kb);
    };
}

#endif
