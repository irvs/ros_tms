class CMultipleParticleFilter
{
public:
  CMultipleParticleFilter();
  virtual ~CMultipleParticleFilter();

  void update(CLaser *Laser);

public:
  std::vector<CPF> m_ParticleFilter;
  CLaser *m_pLaser;

protected:
  int m_max_ID;
  double m_min_distance;
  double m_initial_dist;
  int m_ID;
};
