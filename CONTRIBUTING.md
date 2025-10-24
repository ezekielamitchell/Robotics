# Contributing to Robotics R&D

Thank you for your interest in contributing to this robotics research and development repository. This document provides guidelines for contributions.

## Getting Started

1. Fork the repository
2. Clone your fork: `git clone https://github.com/YOUR-USERNAME/Robotics.git`
3. Create a feature branch: `git checkout -b feature/your-feature-name`
4. Make your changes
5. Test thoroughly
6. Commit with clear messages
7. Push to your fork
8. Submit a pull request

## Code Standards

### Python Code Style

- Follow PEP 8 style guidelines
- Use meaningful variable and function names
- Maximum line length: 100 characters
- Use type hints for function signatures
- Write docstrings for all public functions and classes

Example:
```python
def calculate_servo_angle(target_position: float, current_position: float) -> float:
    """
    Calculate the required servo angle to reach target position.
    
    Args:
        target_position: Desired position in degrees (0-180)
        current_position: Current servo position in degrees
        
    Returns:
        Required angle change in degrees
        
    Raises:
        ValueError: If positions are out of valid range
    """
    if not (0 <= target_position <= 180):
        raise ValueError(f"Target position {target_position} out of range [0, 180]")
    
    return target_position - current_position
```

### Documentation

- Document all hardware requirements
- Include wiring diagrams or connection instructions
- Provide configuration examples
- Document safety considerations
- Include troubleshooting sections

### Testing

- Write unit tests for critical functions
- Test with actual hardware when possible
- Document test procedures
- Include expected outcomes
- Test edge cases and error conditions

## Safety Requirements

### Hardware Testing

Before submitting code that controls physical hardware:

1. Test with minimal power/movement first
2. Implement and test emergency stop functionality
3. Add position/range validation
4. Document safe operating parameters
5. Include warnings for dangerous operations

### Code Review Checklist

- [ ] Code follows style guidelines
- [ ] Documentation is complete and clear
- [ ] Tests are included and passing
- [ ] Safety mechanisms are implemented
- [ ] No hardcoded credentials or secrets
- [ ] Hardware requirements are documented
- [ ] Commit messages are descriptive

## Commit Messages

Use clear, descriptive commit messages:

```
Good:
- "Add emergency stop function to ResponsiveArm controller"
- "Fix servo jitter in initialization sequence"
- "Update IMX477 camera configuration for low-light"

Avoid:
- "fixed bug"
- "updates"
- "wip"
```

## Pull Request Process

1. Ensure all tests pass
2. Update documentation for any changed functionality
3. Add descriptions of hardware tested on (if applicable)
4. Reference any related issues
5. Wait for code review feedback
6. Address review comments promptly

## Security Guidelines

### Never Commit

- API keys, passwords, or credentials
- Private configuration files
- Proprietary algorithms or trade secrets
- Personal or sensitive data
- ITAR/EAR controlled information

### Security Practices

- Use environment variables for sensitive configuration
- Encrypt data transmission in defense applications
- Validate all user inputs
- Log security-relevant events
- Follow organizational security policies

## Defense and Military Applications

### Export Control

If working with defense-related projects:

- Verify ITAR/EAR compliance before sharing
- Mark controlled information appropriately
- Follow organizational export control procedures
- Consult with compliance officers when uncertain

### Ethical Guidelines

- Ensure applications comply with laws of armed conflict
- Consider dual-use implications
- Document intended use cases
- Report ethical concerns to maintainers

## Project-Specific Guidelines

### Lab/OpenCV_Robotics

- Follow the curriculum structure
- Include learning objectives
- Provide example outputs
- Keep dependencies minimal
- Focus on educational clarity

### Projects/ResponsiveArm.pi

- Test on actual Raspberry Pi hardware
- Document servo specifications used
- Include power consumption data
- Verify I2C communication reliability
- Test emergency stop procedures

### Projects/ThreatDetection.IMX477

- Verify camera calibration accuracy
- Document lighting conditions tested
- Measure and report detection latency
- Test false positive/negative rates
- Ensure secure data handling

## Getting Help

- Open an issue for bugs or questions
- Use discussions for general questions
- Tag maintainers for urgent issues
- Provide detailed reproduction steps
- Include system information and logs

## Code of Conduct

- Be respectful and professional
- Welcome newcomers and provide guidance
- Focus on constructive feedback
- Respect different perspectives and experiences
- Maintain confidentiality of sensitive information

## License

By contributing, you agree that your contributions will be licensed under the same license as the project.

## Recognition

Contributors will be recognized in:
- Project documentation
- Release notes for significant contributions
- Academic publications (with permission)

Thank you for contributing to advancing robotics research and development!
