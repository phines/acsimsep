#include "event.hpp"
#include "../regex/regex.hpp"

using namespace std;

void event_t::print() {
	if( ix()==0 ) printf("Num Time Type Location Amount\n");
	printf("%d %g %d %d %g\n", number, time, type, location, amount );
}

/// read event data from an xml formatted string
bool event_t::read( MyXmlNode & event_node ) {
	MyXmlNode node;
	string tag, content;
	
	// error checking
	err_if( event_node.name()!="event", "This is not an event node." );
	// look through the input string for data
	for( node=event_node.child(); not node.is_null(); node = node.next() ) {
		tag     = node.name();
		content = node.content();
		
		if (tag=="type") {
			if      (regex(content, "start"))          type=EVENT_START;
			else if (regex(content, "finish"))         type=EVENT_FINISH;
			// discrete actions
			else if (regex(content, "remove_branch"))  type=REMOVE_BRANCH;
			else if (regex(content, "remove_gen"))     type=REMOVE_GEN;
			else if (regex(content, "remove_load"))    type=REMOVE_LOAD;
			else if (regex(content, "restore_branch")) type=RESTORE_BRANCH;
			else if (regex(content, "restore_gen"))    type=RESTORE_GEN;
			else if (regex(content, "restore_load"))   type=RESTORE_LOAD;
			// continuous actions
			else if (regex(content, "increase_gen"))   type=INCREASE_GEN;
			else if (regex(content, "decrease_gen"))   type=DECREASE_GEN;
			else if (regex(content, "scale_gen"))      type=SCALE_GEN;
			else if (regex(content, "perturb_gen"))    type=PERTURB_GEN;
			else if (regex(content, "increase_load"))  type=INCREASE_LOAD;
			else if (regex(content, "decrease_load"))  type=DECREASE_LOAD;
			else if (regex(content, "scale_load"))     type=SCALE_LOAD;
			else if (regex(content, "perturb_load"))   type=PERTURB_LOAD;
			// operator actions
			else if (regex(content, "run_pf"))         type=RUN_PF;
			else if (regex(content, "run_opf"))        type=RUN_OPF;
			else if (regex(content, "run_scopf"))      type=RUN_SCOPF;
			else if (regex(content, "run_se"))         type=RUN_SE;
			else if (regex(content, "run_mpc"))        type=RUN_MPC;
			// other stuff
			else if (regex(content, "change_step_size")) type=CHANGE_STEP_SIZE;
			else error("Unknown event type in data file");
		}
		else if (tag=="time")     time = atof(content.c_str());
		else if (tag=="amount")   amount = atof(content.c_str());
		else if (tag=="location") {
			if (regex(content,"all"))
				location = EVENT_ALL_LOCATIONS;
			else
				location = atoi(content.c_str());
		}
	}
	return true;
}

/// write event data to an xml formatted string
bool event_t::write ( std::string &data ) {
	// variables
	char str[200];
	data.clear();
	// write the beginning tag
	sprintf(str, "<event number=%d>", number);
	data.append(str);
	// write the time
	sprintf(str, "<time>%g</time>", time);
	data.append(str);
	// write the location
	if (location==EVENT_ALL_LOCATIONS)
		sprintf(str, "<location>all</location>");
	else
		sprintf(str, "<location>%d</location>", location);
	data.append(str);
	// write the amount
	sprintf(str, "<amount>%g</amount>", amount);
	data.append(str);
	// write the type
	data.append("<type>");
	switch(type) {
		// start/finish 
		case EVENT_START:    data.append("start");          break;
		case EVENT_FINISH:   data.append("finish");         break;
		// binary operations on elements
		case REMOVE_BRANCH:  data.append("remove_branch");  break;
		case REMOVE_GEN:     data.append("remove_gen");     break;
		case REMOVE_LOAD:    data.append("remove_load");    break;
		case RESTORE_BRANCH: data.append("restore_branch"); break;
		case RESTORE_GEN:    data.append("restore_gen");    break;
		case RESTORE_LOAD:   data.append("restore_load");   break;
		// continuous operations on loads
		case INCREASE_LOAD:  data.append("increase_load");  break;
		case DECREASE_LOAD:  data.append("decrease_load");  break;
		case SCALE_LOAD:     data.append("scale_load");     break;
		case PERTURB_LOAD:   data.append("perturb_load");   break;
		// continuous operations on gens
		case INCREASE_GEN:   data.append("increase_gen");   break;
		case DECREASE_GEN:   data.append("decrease_gen");   break;
		case SCALE_GEN:      data.append("scale_gen");      break;
		case PERTURB_GEN:    data.append("perturb_gen");    break;
		// global operator actions
		case RUN_OPF:        data.append("run_opf");        break;
		case RUN_PF:         data.append("run_pf");         break;
		case RUN_SCOPF:      data.append("run_scopf");      break;
		case RUN_SE:         data.append("run_se");         break;
		case RUN_MPC:        data.append("run_mpc");        break;
		case NO_EVENT_TYPE:  break;
		// other stuff
		case CHANGE_STEP_SIZE: data.append("change_step_size"); break;
	}
	data.append("</type>");
	// append the end tag
	data.append("</event>");
	return true;
}

