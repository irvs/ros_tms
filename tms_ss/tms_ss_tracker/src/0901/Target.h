#pragma once

// CTarget �R�}���h �^�[�Q�b�g

class CTarget
{
public:
	CTarget();
	virtual ~CTarget();

public:
	int id;
	double px;
	double py;

public:
	void SetPosi(double x, double y);
};

