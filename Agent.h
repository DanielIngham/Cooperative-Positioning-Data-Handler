#pragma once

#include <ostream>
namespace Data {

class Agent {
public:
  enum class Type { ROBOT, LANDMARK };

  Agent(unsigned short id, unsigned short barcode, Type type);
  Agent(Agent &&) = default;
  Agent(const Agent &) = delete;
  Agent &operator=(Agent &&) = delete;
  Agent &operator=(const Agent &) = delete;
  virtual ~Agent() = default;

  struct Identifier {
  protected:
    friend Agent;

    const unsigned short value_;

  public:
    Identifier(unsigned short barcode) : value_{barcode} {};

    bool operator==(const Identifier &other) const {
      return value_ == other.value_;
    }

    bool operator<(const Identifier &other) const {
      return value_ < other.value_;
    }

    operator std::string() const { return std::to_string(value_); }

    friend std::ostream &operator<<(std::ostream &os,
                                    const Identifier &barcode) {
      return os << barcode.value_;
    }

    friend std::string operator+(const std::string &s,
                                 const Identifier &barcode) {
      return s + std::to_string(barcode.value_);
    }
  };

  struct ID : Identifier {};
  struct Barcode : Identifier {};

  const ID id() const;
  const Barcode barcode() const;
  const Type type() const;

private:
  /**
   * @brief Numerical identifier for the robot.
   * @note The handler starts this index at 1. Therefore the first robot has an
   * ID of 1.
   */
  const ID id_;

  /**
   * @brief  Barcode associated with the robot. This is what the other robots
   * will read during there operation to identify each other.
   */
  const Barcode barcode_;

  const Type type_;
};

} // namespace Data
