#include "rosplan_planning_system/FFPlanParserForEsterel.h"

#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <stdlib.h>
#include <algorithm>
#include <ctype.h>
#include <string>

/* implementation of rosplan_planning_system::FFPlanParser.h */
namespace KCL_rosplan { namespace ff_esterel {
         
    //---------------------
    // string manipulation
    //---------------------
        
    namespace str_utils {

        void toLowerCase(std::string &str) {
            std::transform(str.begin(), str.end(), str.begin(), tolower);
        }

        unsigned int split(const std::string &txt, std::vector<std::string> &strs, char ch) {
            unsigned int pos = txt.find( ch );
            unsigned int initialPos = 0;
            strs.clear();
            // Decompose statement
            while( pos != std::string::npos && pos < txt.length()) {
                if(txt.substr( initialPos, pos - initialPos + 1 ) !=" ") {
                    std::string s = txt.substr( initialPos, pos - initialPos + 1 );
                    s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
                    strs.push_back(s);
                }
                initialPos = pos + 1;
                pos = txt.find( ch, initialPos );
            }
            // Add the last one
            strs.push_back( txt.substr( initialPos, txt.size() - initialPos ) );
            return strs.size();
        }

    }

    using str_utils::toLowerCase;
    using str_utils::split;
    
    //---------------------
    // Fast Forward parser 
    //---------------------
        
    FFPlanParserForEsterel::FFPlanParserForEsterel() 
    {
       	// knowledge interface
		//update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan///update_knowledge_base");

		// create operator observation and parameter maps
		parseDomain(); 
    }
    
    FFPlanParserForEsterel::~FFPlanParserForEsterel() {}

    void FFPlanParserForEsterel::reset() {
        plan_nodes.clear();
        plan_edges.clear();
    }
    
    // parse domain
    //----------------
    
	void FFPlanParserForEsterel::parseDomain() {
		
		ros::NodeHandle nh;

		std::string domain_path;
		nh.param("/rosplan/domain_path", domain_path, std::string("common/domain.pddl"));

		std::ifstream infile((domain_path).c_str());
		std::string line;
		std::string action_name;
		while(!infile.eof()) {

			std::getline(infile, line);
			toLowerCase(line);

			// save last seen action name
			std::string::size_type n = line.find("(:action ");
			if (n != std::string::npos) {
 				n = n + 9;

				action_name = line.substr(n);
				int count = 0;
				for(std::string::iterator it = action_name.begin(); it != action_name.end(); ++it) {
				    if (*it == ' ')
						break;
					count++;
				}

				action_name = action_name.substr(0,count);
			}

			// create parameter map for this action
			operator_parameter_map[action_name];

			// add parameters to the map
			n = line.find(":parameters ");
			if (n != std::string::npos) {
 				n = n + 13;
				std::string params = line.substr(n);
				int count_start = 0;
				int count_length = 0;
				bool finished = false;
				int state = 0;
				while(!finished && !infile.eof()) {
					for(std::string::iterator it = params.begin(); it != params.end(); ++it) {
						switch (state) {
						case 0: // reading parameter
							if (*it == '-' || *it == ' ') {
								// std::cout << params.substr(count_start,count_length) << std::endl;
								operator_parameter_map[action_name].push_back(params.substr(count_start,count_length));
								state++;
							}
							break;
						case 1: // skipping " - "
							if (*it != '-' && *it != ' ') state++;
							break;
						case 2: // skipping type
							if (*it == ' ') state++;
							break;
						case 3: // skipping spaces
							if (*it != ' ') {
								count_start = count_start + count_length;
								count_length = 0;
								state = 0;
							}
							break;
						}

						count_length++;

						if (*it == ')') {
							finished = true;
						}
					}

					if(!finished) {
						std::getline(infile, params);
						toLowerCase(params);
					}
				}
			}

			// add single observation to the observation map
			n = line.find(":observe ");
			std::stringstream obs;
			if (n != std::string::npos) {
 				n = n + 9;
				std::string observation = line.substr(n);
				int count = 0;
				int bracket_depth = 0;
				int last_close = 0;
				bool started = false;
				while((!started || bracket_depth > 0) && !infile.eof()) {
					for(std::string::iterator it = observation.begin(); it != observation.end(); ++it) {
					    if (*it == '(') {
							bracket_depth++;
							started = true;
						} else if (*it == ')') {
							bracket_depth--;
							if(bracket_depth>=0)
								last_close = count;
						}
						count++;
					}
					
					if(bracket_depth>=0) {
						obs << observation;
						std::getline(infile, observation);
						toLowerCase(observation);
					} else {
						obs << observation.substr(0,last_close);
					}
				}

				operator_observation_map[action_name] = obs.str();
			}

		}
	}
	
    // Creating nodes and edges 
	//--------------------------

	void FFPlanParserForEsterel::createNode(std::vector<std::string> &s, const std::string& operator_name, int node_id, PlanningEnvironment &environment, StrlNode& node) {

		// concatenate parameters
		std::stringstream ss;
		ss << operator_name << " ";
		int i=3;
		while (i<s.size() && s[i]!="---") {
			ss << s[i] << " ";
			i++;
		}

		node.node_name = ss.str();
		node.node_id = node_id;
		node.dispatched = false;
		node.completed = false;

		// prepare message
		node.dispatch_msg.action_id = node.node_id;
		node.dispatch_msg.duration = 0.1;
		node.dispatch_msg.dispatch_time = 0;
		node.dispatch_msg.name = operator_name;

		plan_nodes.push_back(&node);
	}

	void FFPlanParserForEsterel::createEdge(std::string &child_cffid, StrlNode &node, StrlEdge &edge) {
					
		// save this parent edge
		edge.signal_type = ACTION;
		std::stringstream ss;
		ss << "e_" << node.node_id;
		edge.edge_name = ss.str();
		edge.sources.push_back(&node);
		edge.active = false;
		plan_edges.push_back(&edge);
		node.output.push_back(&edge);

		// prepare for child
		incoming_edge_map[child_cffid];
		incoming_edge_map[child_cffid].push_back(&edge);
	}
    
    // parse a plan written by FF
    //----------------    
        
    void FFPlanParserForEsterel::preparePlan(std::string &dataPath, PlanningEnvironment &environment, size_t freeActionID) {

        std::string filePath;

        if(dataPath.rfind("/") == dataPath.length()-1) {
            filePath = dataPath + "plan.pddl";
        }
        else {
            filePath = dataPath;
        }
        
        ROS_INFO("KCL: (PS)(FFPlanParser) Plan file: %s. Initial action ID: %zu", filePath.c_str(), freeActionID);
        
        std::ifstream infile(filePath.c_str());
        std::string line;
        bool isPlanParsed = false;        
        bool isPlanFound = false;       

        std::vector<StrlEdge*> branches;
        int curr ,next, nodeCount;
        nodeCount = freeActionID;
        StrlEdge* last_edge = NULL;
        
        reset();
        while(!infile.eof() && !isPlanParsed) {

            std::getline(infile, line);

            std::string whitespaces(" \t\f\v\n\r");
            line.erase(line.find_last_not_of(whitespaces)+1);      
            str_utils::toLowerCase(line);

            // search where actions of the plan start
            if(!isPlanFound) {
                // loop until plan is printed             
                if(line.compare("ff: found legal plan as follows") != 0) {
                    continue;                    
                }                

                // remove empty lines after plan header
                bool isScanning = true;
                std::streampos lastPos;                
                while(isScanning) {                    
                    lastPos = infile.tellg();                
                    std::getline(infile, line);
                    str_utils::toLowerCase(line);                                        
                    isScanning = !infile.eof() && line.empty();                    
                }
                
                // plan is found
                if(!infile.eof()) {
                    isPlanFound = true;                
                    infile.seekg(lastPos);
                }
            }            
            // parse plan actions once header is found                    
            else {
                // plan action
                if(!line.empty()) {

                    rosplan_dispatch_msgs::ActionDispatch msg;
                    std::vector<std::string> s;
                    size_t idx = 0;
                                        
                    // actions look like this:
                    // step    0: MOVE_TO_PLACE B1 C1
                    //         1: FIND_OBJECT V1 
                    str_utils::split(line, s, ' ');
                    if(s[0] == "step") { idx = 1; }

                    std::string action_id = s[idx];                    
                    std::string operator_name = s[idx+1];                    
                                      
                    if( ff_node_map.find(action_id) == ff_node_map.end()) {
                        ff_node_map[action_id] = new StrlNode();
                    }
                    
                    StrlNode* node = ff_node_map[action_id];
                    createNode(s, operator_name, nodeCount, environment, *node);
                    ++nodeCount;
                
                    // complete incoming edges
                    if(incoming_edge_map.find(action_id) != incoming_edge_map.end()) {
                        for(int i=0; i<incoming_edge_map[action_id].size(); i++) {
                            node->input.push_back(incoming_edge_map[action_id][i]);
                            incoming_edge_map[action_id][i]->sinks.push_back(node);
                        }
                    }

                    // collect parameters
                    std::vector<std::string> params;
                    for(size_t pdx=idx+2; pdx<s.size(); pdx++) {                        
                        params.push_back(s[pdx]);
                    }
                        
                    // prepare outgoing edges
                    if(s[s.size()-2].substr(0,4) == "son:") {

                        // single child
                        StrlEdge* edge = new StrlEdge();
                        createEdge(s[s.size()-1], *node, *edge);

                    } else if(s[s.size()-2].substr(0,9) == "falseson:") {

                        // two children of observation
                        StrlEdge* edge = new StrlEdge();
                        createEdge(s[s.size()-1], *node, *edge);
                        preparePDDLObservation(operator_name, params, *edge, true);

                        edge = new StrlEdge();
                        createEdge(s[s.size()-4], *node, *edge);
                        preparePDDLObservation(operator_name, params, *edge, false);
                    }

                    ROS_INFO("KCL: (PS)(FFPlanParser) Generate action - ID: %d, Name: %s", nodeCount, operator_name.c_str());
                    
                }
                // no more actions, so parsing is done
                else                    
                {
                    isPlanParsed = true;
       
                }
            }
            
        }

        produceEsterel();
        infile.close();
    }
  
  	void FFPlanParserForEsterel::preparePDDLConditions(std::string operator_name, std::vector<std::string> parameters, StrlNode &node, PlanningEnvironment &environment) {}
	
	void FFPlanParserForEsterel::preparePDDLObservation(std::string &operator_name, std::vector<std::string> &parameters, StrlEdge &edge, bool isNegative) {

		// generate grounded observation
		std::stringstream grounded_observation;
		std::vector<std::string> observationParams;
		split(operator_observation_map[operator_name], observationParams, ' ');

		// for each parameter in observation
		for(int i=0;i<observationParams.size();i++) {

			// trim trailing bracket
			bool hadBracket = false;
			if(observationParams[i].find(")")!=std::string::npos) {
				observationParams[i] = observationParams[i].substr(0,observationParams[i].find(")"));
				hadBracket = true;
			}

			// find matching object in action parameters
			bool found = false;
			for(int j=0;j<operator_parameter_map[operator_name].size();j++) {
				if(observationParams[i] == operator_parameter_map[operator_name][j]) {
					grounded_observation << " " << parameters[j];
					found = true;
					break;
				}
			}
			// string is not a parameter
			if(!found)
				grounded_observation << " " << observationParams[i];

			if(hadBracket)
				grounded_observation << ")";
		}
		
		// update edges
		edge.signal_type = CONDITION;
		if(isNegative) {
			std::stringstream negedge;
			negedge << "(not " << grounded_observation.str() << ")";
			edge.edge_name = negedge.str();
		} else {
			edge.edge_name = grounded_observation.str();
		}
	}
	
	void FFPlanParserForEsterel::generateFilter(PlanningEnvironment &environment) {
		// do nothing yet
	}
    // Produce Esterel 
    //-------------------

    // output a plan as an Esterel controller
    bool FFPlanParserForEsterel::produceEsterel() {

		// output file
		std::string strl_file;
		ros::NodeHandle nh("~");
		nh.param("strl_file_path", strl_file, std::string("common/plan.strl"));
		
		ROS_INFO("KCL: (POPFEsterelPlanParser) Write the esterel plan: %s", strl_file.c_str());
		
		std::ofstream dest;
		dest.open(strl_file.c_str());

		// main module
		dest << "module plan:" << std::endl;

		// inputs
		dest << "input SOURCE";
		std::vector<StrlNode*>::iterator nit = plan_nodes.begin();
		for(; nit!=plan_nodes.end(); nit++) {
			dest << ", a" << (*nit)->node_id << "_complete";
		}
		dest << std::endl;

		// outputs
		dest << "output SINK";		
		for(nit = plan_nodes.begin(); nit!=plan_nodes.end(); nit++) {
			dest << ", a" << (*nit)->node_id << "_dispatch";
		}
		dest << std::endl;

		// internal signals
		std::vector<StrlEdge*>::iterator eit = plan_edges.begin();
		if(eit!=plan_edges.end()) {
			dest << "signal " << (*eit)->edge_name;
			for(; eit!=plan_edges.end(); eit++) {
				dest << ", " << (*eit)->edge_name;
			}
			dest << " in" << std::endl;
		}

		// run everything
		nit = plan_nodes.begin();
		if(nit!=plan_nodes.end()) {
			dest << "run action" << (*nit)->node_id << std::endl;
			for(; nit!=plan_nodes.end(); nit++) {
				dest << " || action" << (*nit)->node_id << std::endl;
			}
			dest << "end" << std::endl;
		}
		dest << "end module" << std::endl << std::endl;

		// action modules
		nit = plan_nodes.begin();
		for(; nit!=plan_nodes.end(); nit++) {

			dest << "module action" << (*nit)->node_id << ":" << std::endl;

			if((*nit)->input.size() > 0) {
				dest << "input ";
				for(int j=0;j<(*nit)->input.size();j++) {
					if(j>0) dest << ", ";
					dest << (*nit)->input[j]->edge_name;
				}
				dest << ";" << std::endl;
			}

			if((*nit)->output.size() > 0) {
				dest << "output ";
				for(int j=0;j<(*nit)->output.size();j++) {
					if(j>0) dest << ", ";
					dest << (*nit)->output[j]->edge_name;
				}
				dest << ";" << std::endl;
			}

			if((*nit)->input.size() > 0) {
				dest << "  await ";
				for(int j=0;j<(*nit)->input.size();j++) {
					if(j>0) dest << " or ";
					dest << (*nit)->input[j]->edge_name;
				}
				dest << ";" << std::endl;
			}

			dest << "  emit a" << (*nit)->node_id << "_dispatch;" << std::endl;
			dest << "  await a" << (*nit)->node_id << "_complete;" << std::endl;
			
			for(int j=0;j<(*nit)->output.size();j++) {
				dest << "emit " << (*nit)->output[j]->edge_name;
			}
			dest << ";" << std::endl;
			dest << "end module" << std::endl << std::endl;
		}
		dest.close();
	}
    
}} // close namespace
