#ifndef INPUTPARSER_HPP_
#define INPUTPARSER_HPP_

// SOURCE: https://stackoverflow.com/questions/865668/how-to-parse-command-line-arguments-in-c

#include <vector>
#include <string>
#include <algorithm> // for std::find

class InputParser
{
    public:
        InputParser (int argc, const char *argv[])
        {
            for (int i=1; i < argc; ++i)
            {
                this->tokens_.push_back(std::string(argv[i]));
            }
        }
        /// @author iain
        const std::string& getCmdOption(const std::string &option) const
        {
            std::vector<std::string>::const_iterator itr;
            itr =  std::find(this->tokens_.begin(), this->tokens_.end(), option);
            if (itr != this->tokens_.end() && ++itr != this->tokens_.end())
            {
                return *itr;
            }
            static const std::string empty_string("");
            return empty_string;
        }
        /// @author iain
        bool cmdOptionExists(const std::string &option) const
        {
            return std::find(this->tokens_.begin(), this->tokens_.end(), option) != this->tokens_.end();
        }
    private:
        std::vector <std::string> tokens_;
};

#endif // #ifndef INPUTPARSER_HPP_
