/**
* Parses the output of Fast Fordward planner and generates a list of ActionDispatch messages.
*/
#include <map>
#include <string>
#include "ros/ros.h"
#include "PlanningEnvironment.h"
#include "PlanParser.h"
#include "EsterelPlan.h"

#ifndef KCL_ff_esterel_plan_parser
#define KCL_ff_esterel_plan_parser

namespace KCL_rosplan { namespace ff_esterel {

    namespace str_utils {
        
        void toLowerCase(std::string &str);
        unsigned int split(const std::string &txt, std::vector<std::string> &strs, char ch);
    }

    class FFPlanParserForEsterel: public PlanParser
    {
    private:                
        std::map<std::string,StrlNode*> ff_node_map;
        std::map<std::string,std::vector<StrlEdge*> > incoming_edge_map;
        std::map<std::string, std::string> operator_observation_map;
        std::map<std::string, std::vector<std::string> > operator_parameter_map;
        
        void parseDomain();
        
        void createNode(std::vector<std::string> &s, const std::string& operator_name, int node_id, PlanningEnvironment &environment, StrlNode& node);
        void createEdge(std::string &child_cffid, StrlNode &node, StrlEdge &edge);

        void preparePDDLObservation(std::string &operator_name, std::vector<std::string> &parameters, StrlEdge &edge, bool isNegative);
        void preparePDDLConditions(std::string operator_name, std::vector<std::string> parameters, StrlNode &node, PlanningEnvironment &environment);
        
public:        
        FFPlanParserForEsterel();
        virtual ~FFPlanParserForEsterel();        
        void reset();
        
        void preparePlan(std::string &dataPath, PlanningEnvironment &environment, size_t freeActionID);
        void generateFilter(PlanningEnvironment &environment);
        
        bool produceEsterel();
        
        std::vector<StrlNode*> plan_nodes;
        std::vector<StrlEdge*> plan_edges;
    };
}}

#endif
