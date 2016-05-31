#include "../regex/regex.hpp"
#include <cstdio>
#include <algorithm>
#include <ctime>
#include "Sim_Solver.hpp"
#include "PowerSystem.hpp"

#define EARLY_FINISH_TIME 20

using namespace std;

/// run the full simulation specified
bool Sim_Solver::run( PowerSystem *PS, char *logFileName )
{
	if ( !init(PS,logFileName) ) return false;
	printf(" Running simulation.\n");
	while ( t_<t_finish_ )
	{
		step();
		printf(" t = %g of %g\n", t_, t_finish_);
	}
	return true;
}

bool Sim_Solver::init( PowerSystem *PS, const char *logFileNameIn ) {
	// variables
	time_t raw_time = time(NULL);
	tm     *ptm = localtime(&raw_time);
	std::string dataFilePrefix;
	char logFileName[200]="temp";
	unsigned  i;
	match_t match;
	
	// save a reference to the power network data
	PS_ = PS;
	// find the prefix for the data file
	if ( regex(PS_->dataFileName,"(.*?)[.]xml", match) ) {
		dataFilePrefix = match[1];
	} else {
		dataFilePrefix = PS_->dataFileName;
	}
	// check the file name, create one if not specified
	if (logFileNameIn==NULL) {
		sprintf(logFileName, "%s_%04d_%02d_%02d__%02d_%02d_%02d.csv",
			dataFilePrefix.c_str(), ptm->tm_year+1900, ptm->tm_mon+1, ptm->tm_mday, ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
	}
	// open the log file
	log_ = fopen(logFileName,"w");
	if (log_==NULL) { printf(" Could not open log file\n"); return false; }
	else printf(" Writing simulation results to %s\n", logFileName);
	// print some information about this case to a file
	print_case_info(logFileName);
	// Make sure that there are at least 2 events.
	if ( PS_->nEvent() < 2 ) {
		printf ("There must be at least 2 events.\n");
		return false;
	}
	
	// sort the events
	sort ( PS_->event.begin(), PS_->event.end() );
	// check to make sure that the first event is start
	if ( PS_->event[0].type != EVENT_START ) { printf(" The event list must begin with a start event\n"); return false; };
	// check to make sure that the last event is finish
	if ( PS_->event[PS->nEvent()-1].type != EVENT_FINISH ) { printf(" The event list must end with a finish event\n"); return false; };
	// record the finish time
	t_finish_ = PS_->event[PS->nEvent()-1].time;
	// set all of the events to not implemented
	for (i=0; i<PS_->nEvent(); i++) {
		PS_->event[i].implemented=false;
		//printf("event type = %d\n", PS_->event[i].type);
	}
	// initialize class variables
	nextEvent_ = &PS_->event[0];
	t_ = nextEvent_->time;
	k_ = 0;
	t_0_ = t_;
	// run an initial power flow to get the first state
	if (!PS_->runPowerFlow()) return false;
	// write a header to the log file
	log_header();
	// record the initial state
	log();
	return true;
}

/// take one step toward the finish, return false when finished, or a problem occurs,
///   or if we determine that nothing else interesting will happen
bool Sim_Solver::step( double dt, string &msg ) {
	unsigned i, nViolations;
	bool finished;
	// check to make sure that we are not at the end
	if ( PS_->nEvent()<2 ) return false;
	// skip ahead to the first non implemented event
	while ( nextEvent_->implemented && nextEvent_->type!=EVENT_FINISH ) {
		nextEvent_++;
	}
	// check for any events to implement during this time period
	while ( nextEvent_->time <= t_ ) {
		if ( nextEvent_->implemented ) {
			nextEvent_++; // not sure how this could happen, but in case it does.
		} else {
			// implement the event
			implement( nextEvent_->type, nextEvent_->location, nextEvent_->amount, msg );
			nextEvent_->implemented=true;
			nextEvent_++;
		}
	}
	// deal with the step size
	// if dt==NO_STEP_SIZE
	if ( abs( dt - NO_STEP_SIZE ) < 1e-9 ) dt = dt_;
	// calculate the next system state
	if ( !PS_->runPowerFlow( true ) ) {
		PS_->set_blackout();
		msg.append("Could not solve power flow. Entire system is now blackout. ");
		printf( "%s\n", msg.c_str() );
		return false;
	}
	
	// update the relays
	bool status_change=false;
	for(i=0;i<PS_->nBranch();i++) {
		if( PS_->branch[i].update_relays( dt ) ) {
			PS_->invalidate();
			status_change = true;
		}
	}
	// increment the step counter
	k_++;
	// increment the time recorder
	t_ += dt;
	// log the results
	log();
	// make a note if any components are stressed
	nViolations = note_stressed_components( msg );
	// choose the return value
	finished = ( t_ >= t_finish_ ) or ( nViolations==0 and t_ > EARLY_FINISH_TIME );
	return ( not finished );
}

/// write a header for the log file
void Sim_Solver::log_header() {
	unsigned i;
	//time_t theTime=time(NULL);
	// print the 
	//fprintf(log_, "%% Simulation results, from file \"%s\", %s, run at %s\n", 
	//	PS_->dataFileName.c_str(), PS_->description.c_str(), asctime(localtime(&theTime)) );
	// time
	fprintf(log_, "time, ");
	// print the voltage magnitude headers
	for (i=0; i<PS_->nBus(); i++)
		fprintf(log_,"Vmag_%03d, ",PS_->bus[i].number);
	// print the voltage angle headers
	for (i=0; i<PS_->nBus(); i++)
		fprintf(log_,"Vang_%03d, ",PS_->bus[i].number);
	// branch currents
	for (i=0; i<PS_->nBranch(); i++)
		fprintf(log_,"If_%03d, ",PS_->branch[i].number);
	for (i=0; i<PS_->nBranch(); i++)
		fprintf(log_,"It_%03d, ",PS_->branch[i].number);
	// load
	for (i=0; i<PS_->nLoad(); i++)
		fprintf(log_,"Sload_%03d, ",PS_->load[i].number);
	// gen
	for (i=0; i<PS_->nGen(); i++)
		fprintf(log_,"Sgen_%03d, ",PS_->gen[i].number);
	fprintf(log_,"\n");
}

/// Log the state to a file
bool Sim_Solver::log() {
	unsigned i;
	// time
	fprintf(log_, "%g, ", t_);
	// print the voltage magnitude
	for (i=0; i<PS_->nBus(); i++)
		fprintf(log_,"%g, ", PS_->bus[i].Vmag);
	// print the voltage angle
	for (i=0; i<PS_->nBus(); i++)
		fprintf(log_,"%g, ", PS_->bus[i].Vang);
	// branch currents
	for (i=0; i<PS_->nBranch(); i++)
		fprintf(log_,"%g%+gj, ", PS_->branch[i].If_re()*PS_->branch[i].status(), PS_->branch[i].If_im()*PS_->branch[i].status());
	for (i=0; i<PS_->nBranch(); i++)
		fprintf(log_,"%g%+gj, ", PS_->branch[i].It_re()*PS_->branch[i].status(), PS_->branch[i].It_im()*PS_->branch[i].status());
	// load
	for (i=0; i<PS_->nLoad(); i++)
		fprintf(log_,"%g%+gj, ", PS_->load[i].Pd*PS_->load[i].status, PS_->load[i].Qd*PS_->load[i].status);
	// gen
	for (i=0; i<PS_->nGen(); i++)
		fprintf(log_,"%g%+gj, ", PS_->gen[i].Pg *PS_->gen[i].status , PS_->gen[i].Qg *PS_->gen[i].status);
	fprintf(log_,"\n");
	return true;
}

/// implement an event of the specified type, location, and amount
bool Sim_Solver::implement (int type_in, int location, double amount, string &msg) {
	// 
	event_type_e type = (event_type_e) type_in;
	char text[200];
	unsigned i;
	double factor;
	
	switch (type) {
		// binary operations
		case REMOVE_BRANCH:
			PS_->removeBranch(location);
			sprintf(text,"Event occurred: Branch %d removed. ", location);
			break;
		case RESTORE_BRANCH:
			PS_->restoreBranch(location);
			sprintf(text,"Event occurred: Branch %d restored. ", location);
			break;
		case REMOVE_GEN:
			PS_->gen(location).status=OFF;
			sprintf(text,"Event occurred: Generator %d removed from service. ", location);
			break;
		case RESTORE_GEN:
			PS_->gen(location).status=ON;
			sprintf(text,"Event occurred: Generator %d restored to service. ", location);
			break;
		case REMOVE_LOAD:
			PS_->load(location).status=OFF;
			sprintf(text,"Event occurred: Load %d removed from service. ", location);
			break;
		case RESTORE_LOAD:
			PS_->load(location).status=ON;
			sprintf(text,"Event occurred: Load %d restored to service. ", location);
			break;
		// continuous gen operations
		case INCREASE_GEN:
			if (location==EVENT_ALL_LOCATIONS) {
				for(i=0;i<PS_->nGen();i++) PS_->gen[i].Pg += amount;
				sprintf(text,"Event occurred: All generators increased (decreased) by %g MW. ", amount);
			} else {
				PS_->gen(location).Pg += amount;
				sprintf(text,"Event occurred: Generator %d increased (decreased) by %g MW. ", location, amount);
			}
			break;
		case DECREASE_GEN:
			implement(INCREASE_GEN, location, -amount, msg);
			break;
		case PERTURB_GEN:
			if (location==EVENT_ALL_LOCATIONS) {
				for( i=0; i<PS_->nGen(); i++ ) {
					PS_->gen[i].Pg *= rng_.rand(NORMAL,1,amount);
				}
				sprintf(text,"Event occurred: All generators randomly perturbed by about %g %%. ", amount*100);
			} else {
				PS_->gen(location).Pg *= rng_.rand(NORMAL,1,amount);
				sprintf(text,"Event occurred: Generator %d randomly perturbed by about %g %%. ", location, amount*100);
			}
			break;
		case SCALE_GEN:
			if (location==EVENT_ALL_LOCATIONS) {
				for ( i=0; i<PS_->nGen(); i++ ) {
					PS_->gen[i].Pg *= amount;
				}
				sprintf(text,"Event occurred: All generators scaled by %g %%. ", amount*100);
			} else {
				PS_->gen(location).Pg *= amount;
				sprintf(text,"Event occurred: Generator %d scaled by about %g %%. ", location, amount*100);
			}
			break;
		// continuous load operations
		case INCREASE_LOAD:
			if (location==EVENT_ALL_LOCATIONS) {
				for ( i=0; i<PS_->nLoad(); i++ ) {
					PS_->load[i].change(amount);
				}
				sprintf(text,"Event occurred: All loads increased by %g MW. ", amount*100);
			} else {
				PS_->load(location).change(amount);
				PS_->load(location).Pd += amount;
				sprintf(text,"Event occurred: Load %d increased by %g MW. ", location, amount*100);
			}
			break;
		case DECREASE_LOAD:
			implement(INCREASE_LOAD, location, -amount, msg);
			break;
		case SCALE_LOAD:
			if (location==EVENT_ALL_LOCATIONS) {
				for ( i=0; i<PS_->nLoad(); i++ ) {
					PS_->load[i].Qd *= amount;
					PS_->load[i].Pd *= amount;
				}
				sprintf(text,"Event occurred: All loads scaled by %g %%. ", amount*100);
			} else {
				PS_->load(location).Qd *= amount;
				PS_->load(location).Pd *= amount;
				sprintf(text,"Event occurred: Load %d scaled by %g %%. ", location, amount*100);
			}
			break;
		case PERTURB_LOAD:
			if (location==EVENT_ALL_LOCATIONS) {
				for( i=0; i<PS_->nLoad(); i++ ) {
					factor = rng_.rand(NORMAL,1,amount);
					PS_->load[i].Qd *= factor;
					PS_->load[i].Pd *= factor;
				}
				sprintf(text,"Event occurred: All loads randomly perturbed by about %g %%. ", amount*100);
			} else {
				factor = rng_.rand(NORMAL,1,amount);
				PS_->load(location).Qd *= factor;
				PS_->load(location).Pd *= factor;
				sprintf(text,"Event occurred: Load %d randomly perturbed by %g %%. ", location, amount*100);
			}
			break;
		case CHANGE_STEP_SIZE:
			dt_ = amount;
			sprintf(text,"Event occurred: Step size changed to %g seconds. ", amount);
			break;
		case EVENT_START:
			sprintf(text,"Starting event sequence. ");
			break;
		case EVENT_FINISH:
			sprintf(text,"Finishing event sequence. ");
			return false;
			break;
		case RUN_OPF:
			break;
		case RUN_SCOPF:
			break;
		case RUN_PF:
			break;
		case RUN_SE:
			break;
		case RUN_MPC:
			break;
		case NO_EVENT_TYPE:
			break;
		default:
			break;
	}
	msg.append(text);
	return true;
}

void Sim_Solver::print_case_info(const char *logFileName) {
	char filename[200];
	std::string prefix;
	FILE *file;
	unsigned i;
	match_t match;
	
	// choose the file name
	if (regex(logFileName, "(.*?)[.]csv", match))  {
		prefix = match[1];
	} else {
		prefix = logFileName;
	}
	sprintf(filename,"%s.m",prefix.c_str());
	// open the file
	file = fopen(filename,"w");
	
	// record the header
	fprintf(file, "%% m file automatically generated by Sim_Solver\n");
	fprintf(file, "%% This file is used to produce plots from the data file specified below:\n");
	fprintf(file, "csv_file = '%s';\n", logFileName);
	// record the size of the network
	fprintf(file, "nBus = %d;\n",    PS_->nBus());
	fprintf(file, "nBranch = %d;\n", PS_->nBranch());
	fprintf(file, "nLoad = %d;\n",   PS_->nLoad());
	fprintf(file, "nGen = %d;\n",    PS_->nGen());
	
	// print the branch ratings
	fprintf(file, "rateA = [");
	for (i=0; i<PS_->nBranch(); i++) {
		fprintf(file, "%g ", PS_->branch[i].rateA);
	}
	fprintf(file, "];\n");
	fprintf(file, "rateB = [");
	for (i=0; i<PS_->nBranch(); i++) {
		fprintf(file, "%g ", PS_->branch[i].rateB);
	}
	fprintf(file, "];\n");
	fprintf(file, "rateC = [");
	for (i=0; i<PS_->nBranch(); i++) {
		fprintf(file, "%g ", PS_->branch[i].rateC);
	}
	fprintf(file, "];\n");
	
	// print the voltage min/max
	fprintf(file, "Vmin = [");
	for (i=0; i<PS_->nBus(); i++) {
		fprintf(file, "%g ", PS_->bus[i].Vmin);
	}
	fprintf(file, "];\n");
	fprintf(file, "Vmax = [");
	for (i=0; i<PS_->nBus(); i++) {
		fprintf(file, "%g ", PS_->bus[i].Vmax);
	}
	fprintf(file, "];\n");
	
	// print the load values
	fprintf(file, "load_value = [");
	for( i=0; i<PS_->nLoad(); i++ ) {
		fprintf( file, "%g ", PS_->load[i].value );
	}
	fprintf(file, "];\n");
	
	// print the gen ramp up costs
	fprintf(file, "gen_ramp_up_cost = [");
	for( i=0; i<PS_->nGen(); i++ ) {
		fprintf( file, "%g ", PS_->gen[i].RUC() );
	}
	fprintf(file, "];\n");
	// print the gen ramp down costs
	fprintf(file, "gen_ramp_down_cost = [");
	for( i=0; i<PS_->nGen(); i++ ) {
		fprintf( file, "%g ", PS_->gen[i].RDC() );
	}
	fprintf(file, "];\n");
	
	fclose(file);
	
	return;
}

int Sim_Solver::note_stressed_components(std::string &note) {
	return PS_->print_violations( note );
}



