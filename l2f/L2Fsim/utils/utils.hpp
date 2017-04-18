#ifndef PI
#define PI 3.14159265358979323846
#endif

#ifndef L2FSIM_UTILS_HPP_
#define L2FSIM_UTILS_HPP_

#include <random>
#include <fstream>
#include <iostream>
#include <algorithm>

namespace L2Fsim {

    /**
     * Sort the indices of the input vector, not depending on its content type
     * @param {const std::vector<double> &} v; input vector
     * @param {std::vector<unsigned int> &} up_ind; indices of the elements of v reaching the maximum values
     * @param {std::vector<unsigned int> &} dw_ind; indices of the other elements of v
     */
    template <class content_type>
    inline void sort_indices(const std::vector<content_type> &v,
        std::vector<unsigned int> &up_ind,
        std::vector<unsigned int> &dw_ind)
    {
        double maxval = *std::max_element(v.begin(),v.end());
        for (unsigned int j=0; j<v.size(); ++j) {
            if(v[j]<maxval) {dw_ind.push_back(j);}
            else {up_ind.push_back(j);}
        }
    }

    /**
     * Overload of the previous function only retrieving the indices of the elements of v reaching the maximum values
     * @param {const std::vector<double> &} v; input vector
     * @param {std::vector<unsigned int> &} up_ind; indices of the elements of v reaching the maximum values
     */
    template <class content_type>
    inline void sort_indices(const std::vector<content_type> &v,
        std::vector<unsigned int> &up_ind)
    {
        double maxval = *std::max_element(v.begin(),v.end());
        for (unsigned int j=0; j<v.size(); ++j) {
            if(v[j]>=maxval) {up_ind.push_back(j);}
        }
    }

    /**
     * Pick a random indice of the input vector, not depending on its content type
     * @param {const std::vector<content_type> &} v; input vector
     * @return {int} a random indice
     * @note initialize a random seed when executing your program
     */
    template <class content_type>
    inline int rand_indice(const std::vector<content_type> &v) {
        if(v.size()==0)
        {std::cout<<"Error in function 'int L2Fsim::rand_indice': argument has size 0"<<std::endl;}
        return rand() % v.size();
    }

    /**
     * Pick a random element of the input vector, not depending on its content type
     * @param {const std::vector<content_type> &} v; input vector
     * @return {int} a random indice
     * @note initialize a random seed when executing your program
     */
    template <class content_type>
    inline content_type rand_element(const std::vector<content_type> &v) {
        return v.at(rand_indice(v));
    }

    inline double sgn(double T) {
        if (T > 0){ return 1.0;}
        if (T < 0){ return -1.0;}
        if (T == 0){ return 1.0;}
        return 0;
    }

    /*--------------------------------------
     -------------   RANDOM   --------------
     -------------------------------------*/

    // return a double between double "a" and double "b"
    inline double rand_double(double a, double b)
    {
        return ( rand()/(double)RAND_MAX ) * (b-a) + a;
    }

    // return an int between int "a" and int "b"
    // implicitly a<b
    inline int rand_int(int a, int b)
    {
        return (rand()%(b-a) +a);
    }

    // produce a number following a normal distribution
    inline double normalLaw()
    {
        double W,V1,V2;
        do
        {
            double U1=rand_double(0., 1.);
            double U2=rand_double(0., 1.);
            V1=2*U1-1;
            V2=2*U2-1;
            W=V1*V1+V2*V2;
        }while(W>1);
        double W_function=sqrt(-2*log(W)/W);
        return V1*W_function;
    }

    /**
     * Save the state into a file
     */
    inline void save_state_into_file(std::string filename, std::vector<double> &state,double time) {
        std::ofstream outfile;
        if (time==0.){outfile.open(filename);}
        else{outfile.open(filename,std::ios::app);}
        // La taille d'un vecteur
        int size = state.size();

        std::string info="";

        for(int i = 0;i<size;i++)
        {
            info = info + std::to_string(state.at(i)) + " ";
        }

        info = info + std::to_string(time);
        outfile << info << std::endl;
        outfile.close();
    }

    inline void save_energy_into_file(std::string filename,double time,double dt,
                                       std::vector<double> &obs_prec,
                                       std::vector<double> &obs)
    {
        const static double g = 9.81;
        double h0 = obs_prec[0];
        double h1 = obs[0];
        double V0 = obs_prec[1];
        double V1 = obs[1];
        /*
        double energie = masse * (g*h0 + V0*V0/2);
        double varioEtienne =  masse * (g*(h1-h0) + V0*(V1-V0));
        double varioAdrien = g * masse *(h1 - h0) + *masse/2*(V1*V1 - V0*V0);
        double varioLouis = ((h1-h0) + 0.5*(V1*V1-V0*V0)/g) /dt;
        */

        double dEc = ((V1*V1-V0*V0) / (2.*g)) /dt;
        double dEp =  (h1-h0)/dt ;
        double dEtot = dEc + dEp;

        std::ofstream outfile;
        if (time==0.)
        {
            outfile.open(filename);
            std::string header="time V0 V1 dEc dEp dEtot";
            outfile << header << std::endl;
        }
        else{outfile.open(filename,std::ios::app);}

        std::string info=std::to_string(time)+" "
                        +std::to_string(V0)+" "
                        +std::to_string(V1)+" "
                        +std::to_string(dEc)+" "
                        +std::to_string(dEp)+" "
                        +std::to_string(dEtot);

        outfile << info << std::endl;

        outfile.close();
    }


    inline void save_Qtable(std::string filename,std::vector<double> &state,std::vector<double> & Qtable)
    {
        std::ofstream outfile;
        outfile.open(filename);

        // La taille d'un vecteur
        int size = state.size();

        std::string info = "";

        for(int i = 0;i<size;i++)
        {
            info = info + std::to_string(Qtable.at(i)) + " ";
            if (i==size-1)
            {
                info = info + std::to_string(Qtable.at(i)) + "  ";
            }
        }

        outfile << info << std::endl;
        outfile.close();
    }

    inline void load_Qtable(std::string filename,std::vector<double> & Qtable){
        std::ifstream file(filename, std::ios::in);
        std::string lign;
        getline(file,lign);
        while(lign!=" ")
        {
            unsigned long cut = lign.find(" ");
            unsigned long rest =lign.size()-cut;
            double num = stod(lign.substr(0,cut));
            lign = lign.substr(cut+1,rest);
            Qtable.push_back(num);
        }
    }

    inline double sigmoid(double x,double a,double c)
    {
        double exp_value;
        float return_value;

        /*** Exponential calculation ***/
        exp_value = exp( -1/a*(x-c));

        /*** Final sigmoid value ***/
        return_value = 1 / (1 + exp_value);

        return return_value;
    }

}

#endif
