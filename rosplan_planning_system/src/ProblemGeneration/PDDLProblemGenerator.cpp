
#include <rosplan_planning_system/ProblemGeneration/PDDLProblemGenerator.h>

namespace KCL_rosplan {

    PDDLProblemGenerator::PDDLProblemGenerator(const std::string& kb)
        : ProblemGenerator(kb) {            
    };
    
    /*--------*/
    /* header */
    /*--------*/

    void PDDLProblemGenerator::makeHeader(std::ofstream &pFile) {

        // setup service calls
        ros::NodeHandle nh;

        ros::ServiceClient getNameClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainNameService>(domain_name_service);
        ros::ServiceClient getTypesClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainTypeService>(domain_type_service);
        ros::ServiceClient getInstancesClient = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>(state_instance_service);

        // get domain name
        rosplan_knowledge_msgs::GetDomainNameService nameSrv;
        if (!getNameClient.call(nameSrv)) {
            ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s", domain_name_service.c_str());
        }

        pFile << "(define (problem task)" << std::endl;
        pFile << "(:domain " << nameSrv.response.domain_name << ")" << std::endl;

        /* objects */
        pFile << "(:objects" << std::endl;

        // get types
        rosplan_knowledge_msgs::GetDomainTypeService typeSrv;
        if (!getTypesClient.call(typeSrv)) {
            ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s", domain_type_service.c_str());
        }

        // get instances of each type
        for(size_t t=0; t<typeSrv.response.types.size(); t++) {

            rosplan_knowledge_msgs::GetInstanceService instanceSrv;
            instanceSrv.request.type_name = typeSrv.response.types[t];

            if (!getInstancesClient.call(instanceSrv)) {
                ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s: %s", state_instance_service.c_str(), instanceSrv.request.type_name.c_str());
            } else {
                if(instanceSrv.response.instances.size() == 0) continue;
                pFile << "    ";
                for(size_t i=0;i<instanceSrv.response.instances.size();i++) {
                    pFile << instanceSrv.response.instances[i] << " ";
                }
                pFile << "- " << typeSrv.response.types[t] << std::endl;
            }
        }

        pFile << ")" << std::endl;
    }

     void PDDLProblemGenerator::makeFooter(std::ofstream &pFile) {
        pFile << ")" << std::endl;
     }
       
    bool PDDLProblemGenerator::preparePredicateParameters(DomainFormulas &domainFormulas,
                                                          rosplan_knowledge_msgs::KnowledgeItem &kwItem,
                                                          std::ofstream &pFile) {
        
        class Condition 
        {
        public:
            Condition(std::string attributeName)
                : _attributeName(attributeName) {                            
            }
            
            bool operator()(rosplan_knowledge_msgs::DomainFormula domainFormula) const {
                return domainFormula.name == _attributeName;
            }
            
        protected:
            std::string _attributeName;
        };
                
        bool success = true;
        DomainFormulas::iterator domainIterator;
        domainIterator = std::find_if(domainFormulas.begin(), domainFormulas.end(), Condition(kwItem.attribute_name));

        if(domainIterator != domainFormulas.end()) {                        
            ROS_DEBUG("KCL: (PDDLProblemGenerator) Goal (%s) found in domain", kwItem.attribute_name.c_str());
            ROS_DEBUG("KCL: (PDDLProblemGenerator) Goal Parameters - Num: %zu", kwItem.values.size());
            ROS_DEBUG("KCL: (PDDLProblemGenerator) Domain Parameters - Num: %zu", domainIterator->typed_parameters.size());
            
            for(size_t j = 0; j < domainIterator->typed_parameters.size(); ++j) {
                
                ROS_DEBUG("KCL: (PDDLProblemGenerator) Domain param (%s)", domainIterator->typed_parameters[j].key.c_str());
                
                bool parameterExistsInDomain = false;                                
                for(size_t k = 0; k < kwItem.values.size(); ++k) {
                    
                    const int ARE_EQUAL = 0;
                    if(ARE_EQUAL == kwItem.values[k].key.compare(domainIterator->typed_parameters[j].key)) {
                        ROS_DEBUG("KCL: (PDDLProblemGenerator) Goal param (%s) FOUND in domain (%s)", 
                                    kwItem.values[k].key.c_str(), domainIterator->typed_parameters[j].key.c_str());                                
                        pFile << " " << kwItem.values[k].value;
                        parameterExistsInDomain = true;
                        break;
                    } else {
                        ROS_DEBUG("KCL: (PDDLProblemGenerator) Goal param (%s) not found in domain (%s)", 
                                    kwItem.values[k].key.c_str(), domainIterator->typed_parameters[j].key.c_str());                        
                    }
                }
                
                success = success && parameterExistsInDomain;
            }
        }
        
        ROS_INFO("KCL: (PDDLProblemGenerator) Is goal (%s) found in domain? %d", kwItem.attribute_name.c_str(),
            success);
        
        return success;
    }

    /*---------------*/
    /* initial state */
    /*---------------*/

    bool PDDLProblemGenerator::getPropositions(ros::Time &time, std::ofstream &pFile) {

        bool success = true;
        
        ros::NodeHandle nh;
        ros::ServiceClient getDomainPropsClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>(domain_predicate_service);
        ros::ServiceClient getPropsClient = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(state_proposition_service);
        ros::ServiceClient getTILsClient = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(state_timed_knowledge_service);
        
        rosplan_knowledge_msgs::GetDomainAttributeService domainAttrSrv;        
        if (getDomainPropsClient.call(domainAttrSrv)) {            
            
            std::vector<rosplan_knowledge_msgs::DomainFormula> domainAttributes = domainAttrSrv.response.items;            
            std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator domainIterator = domainAttributes.begin();
            
            for(; domainIterator != domainAttributes.end(); ++domainIterator) {

                rosplan_knowledge_msgs::GetAttributeService attrSrv;
                attrSrv.request.predicate_name = domainIterator->name;
                
                if (!getPropsClient.call(attrSrv)) {
                    ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s: %s", state_proposition_service.c_str(), 
                              attrSrv.request.predicate_name.c_str());
                    success = false;
                    break;
                } else {

                    std::vector<rosplan_knowledge_msgs::KnowledgeItem> attributes = attrSrv.response.attributes;
                    
                    for(size_t i=0;i<attributes.size();i++) {

                        if(attributes[i].is_negative) {
                            pFile << "    (not (";
                        }
                        else {
                            pFile << "    (";
                        }
                        
                        pFile << attributes[i].attribute_name;
                        
                        /*
                        for(size_t j=0; j<attributes[i].values.size(); j++) {
                            pFile << " " << attributes[i].values[j].value;
                        }
                        */
                                       
                        if (!preparePredicateParameters(domainAttributes, attributes[i], pFile)) {
                            success = false;
                            break;
                        }
                        
                        if(attributes[i].is_negative) {
                            pFile << "))";
                        } else {
                            pFile << ")";
                        }

                        pFile << std::endl;
                    }
                }
                pFile << std::endl;

                attrSrv.request.predicate_name = domainIterator->name;
                attrSrv.response.attributes.clear();
                
                if (!getTILsClient.call(attrSrv)) {
                    ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s: %s", state_timed_knowledge_service.c_str(), 
                              attrSrv.request.predicate_name.c_str());
                    success = false;
                    break;
                } else {
                    if(attrSrv.response.attributes.size() == 0) 
                    {
                        continue;
                    }

                    std::vector<rosplan_knowledge_msgs::KnowledgeItem> attributes = attrSrv.response.attributes;
                    
                    for(size_t i=0;i<attributes.size();i++) {

                        if(attributes[i].is_negative) {
                            pFile << "    (at " << (attributes[i].initial_time - time).toSec() << " (not (";                            
                        }
                        else {
                            pFile << "    (at " << (attributes[i].initial_time - time).toSec() << " (";                            
                        }
                                                
                        pFile << attributes[i].attribute_name;
                        
                        /*
                        for(size_t j=0; j<attributes[i].values.size(); j++) {
                            pFile << " " << attributes[i].values[j].value;
                        }
                        */
                        
                        if (!preparePredicateParameters(domainAttributes, attributes[i], pFile)) {
                            success = false;
                            break;
                        }
                        
                        if(attributes[i].is_negative) {
                            pFile << ")))";
                        } else {
                            pFile << "))";
                        }
                        
                        pFile << std::endl;
                    }
                }
                pFile << std::endl;
            }
        } else {
            ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s", domain_predicate_service.c_str());
            success = false;
        }
        
        return success;
    }
   
   void PDDLProblemGenerator::getFunctions(ros::Time &time, std::ofstream &pFile) {
    
        ros::NodeHandle nh;
        ros::ServiceClient getDomainFuncsClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>(domain_function_service);
        ros::ServiceClient getFuncsClient = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(state_function_service);
        
        rosplan_knowledge_msgs::GetDomainAttributeService domainAttrSrv;
        if (!getDomainFuncsClient.call(domainAttrSrv)) {
            ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s", domain_function_service.c_str());
        } else {

            std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator ait = domainAttrSrv.response.items.begin();
            for(; ait != domainAttrSrv.response.items.end(); ait++) {

                rosplan_knowledge_msgs::GetAttributeService attrSrv;
                attrSrv.request.predicate_name = ait->name;
                if (!getFuncsClient.call(attrSrv)) {
                    ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s: %s", state_function_service.c_str(), attrSrv.request.predicate_name.c_str());
                } else {

                    for(size_t i=0;i<attrSrv.response.attributes.size();i++) {

                        rosplan_knowledge_msgs::KnowledgeItem attr = attrSrv.response.attributes[i];

                        pFile << "    (";

                        if(time < attr.initial_time) {
                            pFile << "at " << (attr.initial_time - time).toSec() << " (";
                        }

                        pFile << "= (";

                        pFile << attr.attribute_name;
                        for(size_t j=0; j<attr.values.size(); j++) {
                            pFile << " " << attr.values[j].value;
                        }

                        pFile << ") " << attr.function_value << ")";

                        if(time < attr.initial_time) {
                            pFile << ")";
                        }

                        pFile << std::endl;
                    }
                }
                pFile << std::endl;
            }
        }
    }

    bool PDDLProblemGenerator::makeInitialState(std::ofstream &pFile) {

        bool success = true;
        
        ros::Time time = ros::Time::now() + ros::Duration(1);        
        pFile << "(:init" << std::endl;
        success = success && getPropositions(time, pFile);
        getFunctions(time, pFile);        
        pFile << ")" << std::endl;
        
        return success;
    }

    /*------*/
    /* goal */
    /*------*/
    
    bool PDDLProblemGenerator::makeGoals(std::ofstream &pFile) {
        
        bool success = true;            

        ros::NodeHandle nh;
        ros::ServiceClient getCurrentGoalsClient = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(state_goal_service);
        ros::ServiceClient getDomainPropsClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>(domain_predicate_service);
        rosplan_knowledge_msgs::GetAttributeService currentGoalSrv;        
        rosplan_knowledge_msgs::GetDomainAttributeService domainAttrSrv;
        
        pFile << "(:goal (and" << std::endl;

        if (getCurrentGoalsClient.call(currentGoalSrv) && getDomainPropsClient.call(domainAttrSrv)) {
            
            std::vector<rosplan_knowledge_msgs::KnowledgeItem> goalAttributes = currentGoalSrv.response.attributes;
            std::vector<rosplan_knowledge_msgs::DomainFormula> domainAttributes = domainAttrSrv.response.items;

            ROS_DEBUG("KCL: (PDDLProblemGenerator) Num goals: %zu", goalAttributes.size());

            //std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator goalsIt = goalAttributes.begin();            
            for(size_t i = 0;i < goalAttributes.size(); ++i) {                
                                
                ROS_DEBUG("KCL: (PDDLProblemGenerator) Goal - Name: %s, Type: %d,  IsNegative: %d", 
                          goalAttributes[i].attribute_name.c_str(), goalAttributes[i].knowledge_type, goalAttributes[i].is_negative);
                
                if(goalAttributes[i].knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FACT) {

                    if(goalAttributes[i].is_negative){
                        pFile << "    (not ("+ goalAttributes[i].attribute_name;                        
                    } else {
                        pFile << "    (" + goalAttributes[i].attribute_name;                        
                    }
                    
                    if (!preparePredicateParameters(domainAttributes, goalAttributes[i], pFile)) {
                        success = false;
                        break;
                    }
                        
                    if(goalAttributes[i].is_negative) {
                        pFile << "))";                        
                    } else {
                        pFile << ")";                        
                    }
                    pFile << std::endl;
                }
                
                if(goalAttributes[i].knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INEQUALITY) {

                    if(goalAttributes[i].is_negative) {
                        pFile << "    (not (";                        
                    } else {
                        pFile << "    (";
                    }

                    switch(goalAttributes[i].ineq.comparison_type){
                        case 0: pFile << "> "; break;
                        case 1: pFile << ">= "; break;
                        case 2: pFile << "<" ; break;
                        case 3: pFile << "<= "; break;
                        case 4: pFile << "=" ; break;
                    }

                    printExpression(pFile, goalAttributes[i].ineq.LHS);
                    printExpression(pFile, goalAttributes[i].ineq.RHS);

                    if(goalAttributes[i].is_negative) {
                        pFile << "))";                        
                    } else {
                        pFile << ")";                        
                    }
                    pFile << std::endl;
                }
            }
        } else {            
            ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s or service %s", 
                      state_goal_service.c_str(), domain_predicate_service.c_str());
            success = false;
        }
        pFile << "))" << std::endl;

        return success;
    }

    /*--------*/
    /* metric */
    /*--------*/

    void PDDLProblemGenerator::makeMetric(std::ofstream &pFile) {

        ros::NodeHandle nh;

        // get current metric
        ros::ServiceClient getCurrentMetricClient = nh.serviceClient<rosplan_knowledge_msgs::GetMetricService>(state_metric_service);
        rosplan_knowledge_msgs::GetMetricService currentMetricSrv;
        if (!getCurrentMetricClient.call(currentMetricSrv)) {

            ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s", state_metric_service.c_str());

        } else {

            rosplan_knowledge_msgs::KnowledgeItem metric = currentMetricSrv.response.metric;
            if (metric.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::EXPRESSION) {
                    
                pFile << "(:metric " << metric.optimization;
                std::vector<int> operand_count;
                printExpression(pFile, metric.expr);

                pFile << ")" << std::endl;
            }
        }
    }

    void PDDLProblemGenerator::printExpression(std::ofstream &pFile, rosplan_knowledge_msgs::ExprComposite & e) {

        std::vector<rosplan_knowledge_msgs::ExprBase> tokens = e.tokens;
        bool second_operand = false;
        int depth = 0;
        for(int i=0; i<tokens.size(); i++) {
            rosplan_knowledge_msgs::ExprBase token = tokens[i];

            pFile << " ";

            switch(token.expr_type) {
            case rosplan_knowledge_msgs::ExprBase::OPERATOR:

                switch(token.op) {
                    case rosplan_knowledge_msgs::ExprBase::ADD: pFile << "(+"; break;
                    case rosplan_knowledge_msgs::ExprBase::SUB: pFile << "(-"; break;
                    case rosplan_knowledge_msgs::ExprBase::MUL: pFile << "(*"; break;
                    case rosplan_knowledge_msgs::ExprBase::DIV: pFile << "(/"; break;
                }
                second_operand = false;
                depth++;
                break;

            case rosplan_knowledge_msgs::ExprBase::CONSTANT:

                pFile << token.constant;
                break;

            case rosplan_knowledge_msgs::ExprBase::FUNCTION:

                pFile << "(" << token.function.name;
                for(size_t j=0; j<token.function.typed_parameters.size(); j++) {
                    pFile << " " << token.function.typed_parameters[j].value;
                }
                pFile << ")";
                break;

            case rosplan_knowledge_msgs::ExprBase::SPECIAL:

                switch(token.special_type) {
                    case rosplan_knowledge_msgs::ExprBase::HASHT:		pFile << "#t";			break;
                    case rosplan_knowledge_msgs::ExprBase::TOTAL_TIME:	pFile << "(total-time)";	break;
                    case rosplan_knowledge_msgs::ExprBase::DURATION:	pFile << "?duration";	break;
                }
                break;
            }

            if(second_operand && token.expr_type!=rosplan_knowledge_msgs::ExprBase::OPERATOR) {
                second_operand = true;
                pFile << ")";
                depth--;
            }

            if(token.expr_type!=rosplan_knowledge_msgs::ExprBase::OPERATOR) {
                second_operand = true;
            }
        }

        while(depth>0) {
            pFile << ")";
            depth--;
        }
    }

    void PDDLProblemGenerator::makeProblem(std::ofstream &pFile) {
        
        bool success = true;
        
        makeHeader(pFile);
        success = success && makeInitialState(pFile);
        success = success && makeGoals(pFile);
        makeMetric(pFile);
        makeFooter(pFile);
    };

} // close namespace
