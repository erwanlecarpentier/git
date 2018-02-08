#ifndef SAVE_HPP_
#define SAVE_HPP_

#include <cassert>

/**
 * @brief Save a vector
 *
 * Save a vector into a file, writing this vector on a single line using the given separator
 * and the given writing mode. Template method.
 * @param {const std::vector<T> &} v; the saved vector
 * @param {const std::string &} output_path; the output path (name of the file)
 * @param {const std::string &} separator; the used separator
 * @param {std::ofstream::openmode} mode; the mode: using 'std::ofstream::out' overrides the
 * output file while using 'std::ofstream::app' appends the vector to the bottom of the file.
 * Default is overriding.
 */
template <class T>
void save_vector(
    const std::vector<T> &v,
    const std::string &output_path,
    const std::string &separator,
	std::ofstream::openmode mode = std::ofstream::out)
{
    std::ofstream outfile;
	outfile.open(output_path,mode);
	for(unsigned int i=0; i<v.size(); ++i) {
		outfile << v[i];
		if(i<v.size()-1) {
            outfile << separator;
		}
	}
	outfile << std::endl;
	outfile.close();
}

/**
 * @brief Save a matrix
 *
 * Save a matrix into a file, writing subsequently each one of its line using the method
 * 'save_vector'. Template method.
 * @param {const std::vector<std::vector<T>> &} m; the saved matrix
 * @param {const std::string &} output_path; the output path (name of the file)
 * @param {const std::string &} separator; the used separator
 * @param {std::ofstream::openmode} mode; the mode: using 'std::ofstream::out' overrides the
 * output file while using 'std::ofstream::app' appends the vector to the bottom of the file.
 * Default is overriding.
 */
template <class T>
void save_matrix(
    const std::vector<std::vector<T>> &m,
    const std::string &output_path,
    const std::string &separator,
	std::ofstream::openmode mode = std::ofstream::out)
{
    for(auto &line : m) {
        save_vector(line,output_path,separator,mode);
    }
}

/**
 * @brief Initialize the backup file
 *
 * Overrides the name of the saved values at the given output path. This is the header used
 * for plotting.
 * @param {const std::vector<std::string> &} names; the names of the saved values
 * @param {const std::string &} output_path; the output path (name of the file)
 * @param {const std::string &} separator; the used separator
 */
void initialize_backup(
    const std::vector<std::string> &names,
    const std::string &output_path,
    const std::string &separator)
{
    save_vector(names,output_path,separator,std::ofstream::out);
}

#endif // SAVE_HPP_
